# =============================================================================
# SPIKE Prime (SPIKE App 3.x) — автономная навигация
# Одометрия + гироскоп + PID (Anti-Windup) + Pure Pursuit
# =============================================================================
# Этот файл — монолит: его целиком копируют в Python-проект SPIKE App 3.x.
# Редактируйте только блок НАСТРОЕК и массив PATH. Остальной код — движок.
#
# Кинематика базы: 2 ведущих колеса + 2 гладких (без резины) для скольжения.
# Угол Theta берётся СТРОГО с гироскопа. Моторы дают только пройденный путь.
# =============================================================================

import math
import motor
import runloop
from hub import motion_sensor, port


# =============================================================================
# НАСТРОЙКИ РОБОТА — РЕДАКТИРУЙТЕ ЭТОТ БЛОК
# =============================================================================

# --- Геометрия колёс (мм) ----------------------------------------------------
# Официальное SPIKE-колесо: 56 мм. Измерьте СВОЁ колесо — см. README.
WHEEL_DIAMETER = 56.0

# Колея: расстояние между СЕРЕДИНАМИ ведущих шин (мм).
# Для Pure Pursuit + гироскопа не участвует в расчёте Theta,
# но нужна, чтобы переводить угловую коррекцию в скорости моторов.
WHEELBASE = 120.0

# --- Порты ведущих моторов ---------------------------------------------------
# Смотрите буквы на хабе. Типичная FLL-схема: левый A, правый B.
LEFT_MOTOR_PORT = port.A
RIGHT_MOTOR_PORT = port.B

# Знак мотора: +1, если положительная скорость крутит колесо ВПЕРЁД.
# У большинства баз левый мотор развёрнут «осью наружу» — тогда LEFT = -1.
LEFT_MOTOR_SIGN = -1
RIGHT_MOTOR_SIGN = 1

# --- Гироскоп ----------------------------------------------------------------
# Лицо хаба, смотрящее ВВЕРХ при езде. TOP = матрица светодиодов сверху.
# Если хаб лежит матрицей вперёд / на боку — смените на FRONT / LEFT / RIGHT.
YAW_FACE = motion_sensor.TOP

# Если робот уходит «зеркально» (поворачивает не в ту сторону) — поставьте -1.
GYRO_SIGN = 1

# --- PID по углу (heading) ---------------------------------------------------
# Начните с Kp, Ki=0, Kd=0. Алгоритм подбора — в README.
PID_KP = 3.0
PID_KI = 0.05
PID_KD = 0.35

# Ограничение интеграла (Anti-Windup, clamp). Единицы: «накопленный градус».
PID_I_LIMIT = 80.0

# Максимальная угловая коррекция, град/с, которую PID может запросить.
PID_OUTPUT_LIMIT = 350.0

# --- Скорость и цикл управления ----------------------------------------------
# Базовая линейная скорость в град/с мотора (не мм/с). SPIKE Medium: до ~1110.
BASE_SPEED = 380
MAX_MOTOR_SPEED = 800
MIN_MOTOR_SPEED = -800

# Период цикла, мс. 10 мс = 100 Гц — баланс точности и нагрузки на хаб.
LOOP_MS = 10

# --- Pure Pursuit ------------------------------------------------------------
# Дистанция «взгляда вперёд» (мм). Больше — плавнее дуги, хуже острые углы.
LOOKAHEAD_MM = 160.0

# Радиус «мы на точке» (мм). Если робот ближе — берём следующую цель.
WAYPOINT_RADIUS_MM = 45.0

# Радиус финиша (мм). Доехали до последней точки — стоп.
GOAL_TOLERANCE_MM = 35.0

# Замедление у финиша: скорость линейно падает, когда цель ближе этого (мм).
SLOWDOWN_MM = 120.0

# --- Траектория --------------------------------------------------------------
# Точки в мм в системе: старт = (0, 0), +X вперёд, +Y влево.
# Первая точка должна быть стартовой позицией робота.
PATH = [
    (0.0, 0.0),
    (400.0, 0.0),
    (400.0, 300.0),
    (750.0, 300.0),
    (750.0, 0.0),
]


# =============================================================================
# УТИЛИТЫ
# =============================================================================

def clamp(value, lo, hi):
    """Ограничивает число отрезком [lo, hi]."""
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def wrap_deg(angle):
    """Нормализует угол в диапазон (-180, 180]. Нужно для перехода через ±180°."""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def hypot(dx, dy):
    """Длина вектора. math.hypot есть в MicroPython, но явная форма читаемее."""
    return math.sqrt(dx * dx + dy * dy)


# =============================================================================
# КЛАСС Odometry
# Distances — с энкодеров моторов. Theta — СТРОГО с гироскопа.
# =============================================================================

class Odometry:
    """
    Позиция робота на плоскости.

    x, y  — мм, начало координат = место, где вызвали reset().
    theta — радианы, 0 = направление в момент reset(), против часовой = плюс.

    Формулы (дифференциальный привод, heading с IMU):
        s_L = (deg_L / 360) * π * D
        s_R = (deg_R / 360) * π * D
        Δs  = (Δs_L + Δs_R) / 2
        θ   = gyro_yaw            # НЕ из (s_R - s_L) / L !
        x  += Δs * cos(θ)
        y  += Δs * sin(θ)

    Почему Theta только с гироскопа:
    гладкие опорные колёса скользят, ведущие проскальзывают на коврике FLL.
    Интеграл разности энкодеров быстро врёт. IMU хаба даёт абсолютный yaw.
    """

    def __init__(self, wheel_diameter_mm):
        self.circumference = math.pi * wheel_diameter_mm
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_left_mm = 0.0
        self._prev_right_mm = 0.0

    def _wheel_mm(self, motor_port, sign):
        """Пройденный путь колеса в мм с учётом знака установки мотора."""
        deg = motor.relative_position(motor_port) * sign
        return (deg / 360.0) * self.circumference

    def reset(self):
        """Обнуляет позу, энкодеры и yaw. Вызывать стоя на старте, хаб неподвижен."""
        motion_sensor.set_yaw_face(YAW_FACE)
        motion_sensor.reset_yaw(0)
        motor.reset_relative_position(LEFT_MOTOR_PORT, 0)
        motor.reset_relative_position(RIGHT_MOTOR_PORT, 0)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_left_mm = 0.0
        self._prev_right_mm = 0.0

    def heading_deg(self):
        """
        Угол курса в градусах из motion_sensor.tilt_angles().

        SPIKE возвращает (yaw, pitch, roll) в ДЕЦИГРАДУСАХ (1 единица = 0.1°).
        Диапазон yaw: примерно -1795 … 1800.
        Документация LEGO: плюс = против часовой, минус = по часовой.
        GYRO_SIGN инвертирует, если хаб перевёрнут.
        """
        yaw_decideg = motion_sensor.tilt_angles()[0]
        return wrap_deg(yaw_decideg * 0.1 * GYRO_SIGN)

    def update(self):
        """Один шаг мёртвого счисления. Вызывать каждый цикл (~10 мс)."""
        left_mm = self._wheel_mm(LEFT_MOTOR_PORT, LEFT_MOTOR_SIGN)
        right_mm = self._wheel_mm(RIGHT_MOTOR_PORT, RIGHT_MOTOR_SIGN)

        ds = ((left_mm - self._prev_left_mm) + (right_mm - self._prev_right_mm)) / 2.0
        self._prev_left_mm = left_mm
        self._prev_right_mm = right_mm

        self.theta = math.radians(self.heading_deg())
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)


# =============================================================================
# КЛАСС PID — регулятор по углу с Anti-Windup
# =============================================================================

class PID:
    """
    Классический PID:

        u = Kp·e + Ki·∫e dt + Kd·de/dt

    Anti-Windup (два слоя):
      1) Conditional integration — интеграл НЕ копит ошибку, если выход
         уже упёрся в лимит и ошибка того же знака (иначе «надувается»).
      2) Clamp интеграла — жёсткий потолок |I| ≤ i_limit.

    На входе — ошибка угла в градусах (уже обёрнутая в ±180).
    На выходе — желаемая угловая скорость (град/с), со знаком:
      плюс  = поворот против часовой (влево при езде вперёд),
      минус = поворот по часовой (вправо).
    """

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit)
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._has_prev = False

    def compute(self, error_deg, dt):
        if dt <= 0.0:
            dt = 0.01

        p_term = self.kp * error_deg

        d_term = 0.0
        if self._has_prev:
            d_term = self.kd * (error_deg - self._prev_error) / dt
        self._prev_error = error_deg
        self._has_prev = True

        # Предварительный выход без нового интеграла — чтобы решить, копить ли I.
        unsaturated = p_term + (self.ki * self._integral) + d_term
        saturated = abs(unsaturated) >= self.out_limit
        same_sign = (unsaturated * error_deg) > 0.0

        if not (saturated and same_sign):
            self._integral += error_deg * dt
            self._integral = clamp(self._integral, -self.i_limit, self.i_limit)

        i_term = self.ki * self._integral
        output = p_term + i_term + d_term
        return clamp(output, -self.out_limit, self.out_limit)


# =============================================================================
# КЛАСС PathFollower — Pure Pursuit по ломаной [(x, y), ...]
# =============================================================================

class PathFollower:
    """
    Pure Pursuit для ломаной из путевых точек.

    Идея: робот всегда целится в точку на траектории на расстоянии
    LOOKAHEAD_MM впереди. PID держит курс на эту точку.

    Алгоритм кадра:
      1. Продвинуть индекс сегмента, если ближайшая вершина уже пройдена.
      2. Найти пересечение окружности радиуса Ld с текущим/следующими
         сегментами. Взять самое дальнее пересечение вдоль пути.
      3. Если пересечений нет — целиться в конец текущего сегмента.
      4. Желаемый курс = atan2(ty - y, tx - x).
      5. Ошибка курса → PID → разность скоростей колёс.

    Финиш: расстояние до ПОСЛЕДНЕЙ точки < GOAL_TOLERANCE_MM.
    """

    def __init__(self, path, lookahead_mm, waypoint_radius_mm, goal_tol_mm):
        if path is None or len(path) < 2:
            raise ValueError("PATH должен содержать минимум 2 точки")
        self.path = path
        self.lookahead = lookahead_mm
        self.waypoint_radius = waypoint_radius_mm
        self.goal_tol = goal_tol_mm
        self.seg_index = 0
        self.finished = False
        self.target_x = path[0][0]
        self.target_y = path[0][1]

    def _advance_segment(self, x, y):
        """Не возвращаемся назад: сегмент считается пройденным у его конца."""
        n = len(self.path)
        while self.seg_index < n - 2:
            end = self.path[self.seg_index + 1]
            if hypot(end[0] - x, end[1] - y) <= self.waypoint_radius:
                self.seg_index += 1
            else:
                break

    def _circle_segment_hits(self, x, y, ax, ay, bx, by):
        """
        Пересечения окружности (центр = робот, R = lookahead)
        с отрезком A→B. Возвращает список t в [0, 1].
        """
        dx = bx - ax
        dy = by - ay
        fx = ax - x
        fy = ay - y

        a = dx * dx + dy * dy
        if a < 1e-9:
            return []

        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - self.lookahead * self.lookahead
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return []

        sqrt_d = math.sqrt(disc)
        hits = []
        for sign in (-1.0, 1.0):
            t = (-b + sign * sqrt_d) / (2.0 * a)
            if 0.0 <= t <= 1.0:
                hits.append(t)
        return hits

    def _lookahead_point(self, x, y):
        """Ищет точку преследования, сканируя сегменты от текущего к концу."""
        best_t = -1.0
        best_i = self.seg_index
        best_x = self.path[self.seg_index + 1][0]
        best_y = self.path[self.seg_index + 1][1]
        found = False

        i = self.seg_index
        while i < len(self.path) - 1:
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            hits = self._circle_segment_hits(x, y, ax, ay, bx, by)
            for t in hits:
                # Берём пересечение дальше вдоль пути (больший индекс, затем t).
                if (not found) or (i > best_i) or (i == best_i and t > best_t):
                    best_t = t
                    best_i = i
                    best_x = ax + t * (bx - ax)
                    best_y = ay + t * (by - ay)
                    found = True
            i += 1

        if found:
            self.target_x = best_x
            self.target_y = best_y
            return

        # Робот слишком далеко от пути или Ld больше оставшегося куска —
        # целимся в конец текущего сегмента (или в финиш).
        end = self.path[self.seg_index + 1]
        self.target_x = end[0]
        self.target_y = end[1]

    def desired_heading_deg(self, x, y):
        """Курс на точку преследования, градусы, CCW от +X."""
        return math.degrees(math.atan2(self.target_y - y, self.target_x - x))

    def dist_to_goal(self, x, y):
        gx, gy = self.path[-1]
        return hypot(gx - x, gy - y)

    def update(self, x, y):
        """
        Обновляет цель. Возвращает True, пока надо ехать.
        """
        if self.finished:
            return False

        if self.dist_to_goal(x, y) <= self.goal_tol:
            self.finished = True
            self.target_x = self.path[-1][0]
            self.target_y = self.path[-1][1]
            return False

        self._advance_segment(x, y)
        self._lookahead_point(x, y)
        return True


# =============================================================================
# ПРИВОД КОЛЁС
# =============================================================================

def apply_tank(base_speed, omega_deg_s):
    """
    Смешивание tank-drive:

        v_L = v - ω * (L / D)
        v_R = v + ω * (L / D)

    omega_deg_s — выход PID (град/с кузова).
    Перевод в град/с мотора: колесо должно проехать дугу (wheelbase/2)*dθ,
    делённую на радиус колеса. Упрощённо используем масштаб WHEELBASE / D.
    """
    scale = WHEELBASE / WHEEL_DIAMETER
    delta = omega_deg_s * scale
    left = base_speed - delta
    right = base_speed + delta

    # Сохраняем отношение скоростей, если упёрлись в лимит мотора.
    peak = max(abs(left), abs(right), 1.0)
    if peak > MAX_MOTOR_SPEED:
        k = MAX_MOTOR_SPEED / peak
        left *= k
        right *= k

    left = clamp(left, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED)
    right = clamp(right, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED)

    motor.run(LEFT_MOTOR_PORT, int(left * LEFT_MOTOR_SIGN))
    motor.run(RIGHT_MOTOR_PORT, int(right * RIGHT_MOTOR_SIGN))


def stop_drive():
    motor.stop(LEFT_MOTOR_PORT)
    motor.stop(RIGHT_MOTOR_PORT)


def cruise_speed(dist_to_goal):
    """Линейное замедление у финиша, чтобы не промазать точку."""
    if dist_to_goal >= SLOWDOWN_MM:
        return float(BASE_SPEED)
    ratio = dist_to_goal / SLOWDOWN_MM
    return BASE_SPEED * clamp(ratio, 0.25, 1.0)


# =============================================================================
# ГЛАВНЫЙ АСИНХРОННЫЙ ЦИКЛ
# =============================================================================

async def main():
    odom = Odometry(WHEEL_DIAMETER)
    pid = PID(PID_KP, PID_KI, PID_KD, PID_I_LIMIT, PID_OUTPUT_LIMIT)
    follower = PathFollower(PATH, LOOKAHEAD_MM, WAYPOINT_RADIUS_MM, GOAL_TOLERANCE_MM)

    # Дать IMU время стабилизироваться. Не двигайте робота в эти 400 мс.
    odom.reset()
    await runloop.sleep_ms(400)
    odom.reset()
    pid.reset()

    dt = LOOP_MS / 1000.0

    while True:
        odom.update()

        if not follower.update(odom.x, odom.y):
            stop_drive()
            break

        heading_error = wrap_deg(follower.desired_heading_deg(odom.x, odom.y) - odom.heading_deg())
        omega = pid.compute(heading_error, dt)
        speed = cruise_speed(follower.dist_to_goal(odom.x, odom.y))
        apply_tank(speed, omega)

        await runloop.sleep_ms(LOOP_MS)

    stop_drive()


# Точка входа SPIKE App 3.x: без этой строки программа не стартует.
runloop.run(main())
