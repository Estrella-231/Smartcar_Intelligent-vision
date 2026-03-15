import gc
import math
import sensor
import time
from machine import UART


# 调试开关：是否在图像上画采样点、是否打印地图和状态、是否只在结果有效时发送串口
DEBUG_DRAW = True
DEBUG_PRINT = True
SEND_ONLY_WHEN_VALID = False
DEBUG_PRINT_EVERY = 30
DEBUG_PRINT_SAMPLES = False
DEBUG_PRINT_MAP = True
DEBUG_DRAW_STATUS = False
DEBUG_FRAME_STATS = False
DEBUG_PRINT_CLASS_COUNTS = True
DEBUG_PRINT_PLAYER_CANDIDATES = True
DEBUG_PRINT_OBJECT_COORDS = True

# 调试绘制参数
# 先恢复为密集绘制，方便整体标定
DRAW_FULL_GRID = True
DRAW_STRIDE = 1
DRAW_POINT_STYLE = "dot"   # "dot" 或 "cross"
DRAW_POINT_SIZE = 1
DRAW_CENTER_MARK = True

# 颜色分类模式：当前默认 RGB，后续可切换到 LAB
COLOR_MODE = "RGB"  # Reserved: "LAB"
USE_WALL_SPECIAL_RULE = True

# 摄像头基础配置
FRAME_PIXFORMAT = sensor.RGB565
FRAME_SIZE = sensor.QVGA
SKIP_FRAMES_MS = 2000
LOCK_AUTO_EXPOSURE = False
EXPOSURE_US = 2000

# 16x12 网格标定参数
# 这几个值是后续现场调试的重点：
# START_X / START_Y 决定左上角第一个采样点位置
# STEP_X / STEP_Y 决定网格间距
# ROTATION 用于修正整体画面轻微旋转
GRID_W = 16
GRID_H = 12
START_X = 20
START_Y = 15
STEP_X = 18.5
STEP_Y = 18.5
ROTATION = 0.0

# 墙体特殊判定参数
# 黑灰墙体的 RGB 三通道接近，因此可用“通道接近度 + 整体亮度偏低”辅助识别
WALL_GRAY_SPREAD_MAX = 36
WALL_GRAY_BRIGHTNESS_MAX = 110

# 玩家特殊判定参数
# 玩家标识由绿色和青色组成，单点取色不稳定，因此使用邻域联合判定
GREEN_R_MAX = 120
GREEN_G_MIN = 90
GREEN_B_MAX = 120
CYAN_R_MAX = 90
CYAN_G_MIN = 90
CYAN_B_MIN = 90
PLAYER_NEIGHBOR_OFFSETS = ((0, 0), (-2, 0), (2, 0), (0, -2), (0, 2))
GREEN_CYAN_MIN_COUNT = 2

# UART 配置
UART_PORT = 1   # User confirmed UART1, TX=C16, RX=C17
UART_BAUDRATE = 115200

# 串口协议配置
# 总长度 199 字节：
# [0]帧头 [1]长度 [2]状态 [3~194]地图 [195]X [196]Y [197]校验 [198]帧尾
FRAME_HEADER = 0xDF
FRAME_TAIL = 0xD0
STATUS_INVALID = 0x00
STATUS_VALID = 0x01
PAYLOAD_LEN = 195
PACKET_LEN = 199

# 地图编码定义
CELL_EMPTY = 0
CELL_WALL = 1
CELL_BOX = 2
CELL_TARGET = 3
CELL_BOMB = 4
CELL_PLAYER = 5

COLOR_REFERENCES = {
    CELL_EMPTY: (33, 12, 255),
    CELL_WALL: (57, 65, 82),
    CELL_TARGET: (231, 0, 255),
    CELL_BOX: (148, 178, 0),
    CELL_BOMB: (255, 24, 74),
    CELL_PLAYER: (16, 154, 0),
}

COLOR_NAMES = {
    CELL_EMPTY: "empty",
    CELL_WALL: "wall",
    CELL_BOX: "box",
    CELL_TARGET: "target",
    CELL_BOMB: "bomb",
    CELL_PLAYER: "player",
}

CELL_CHARS = {
    CELL_EMPTY: ".",
    CELL_WALL: "#",
    CELL_BOX: "$",
    CELL_TARGET: "T",
    CELL_BOMB: "B",
    CELL_PLAYER: "P",
}

debug_frame_count = 0


def init_sensor():
    # 初始化摄像头。
    # OpenART Plus 上不再调用 set_brightness()，只通过曝光参数控制亮度，
    # 避免底层 sensor 驱动对 brightness 寄存器支持不完整时导致白屏。
    sensor.reset()
    sensor.set_pixformat(FRAME_PIXFORMAT)
    sensor.set_framesize(FRAME_SIZE)
    sensor.skip_frames(time=SKIP_FRAMES_MS)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(False)
    if LOCK_AUTO_EXPOSURE:
        sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)


def init_uart():
    # 初始化串口。具体引脚由板端 UART1 默认复用到 C16/C17
    return UART(UART_PORT, baudrate=UART_BAUDRATE)


def build_sample_points():
    # 依据标定参数预先生成 16x12 共 192 个采样点
    # 顺序严格采用“先行后列”：
    # 先生成第 0 行的 16 个点，再生成第 1 行，直到第 11 行
    points = []
    cos_rot = math.cos(ROTATION)
    sin_rot = math.sin(ROTATION)

    for row in range(GRID_H):
        for col in range(GRID_W):
            # 先计算未旋转时的理论偏移
            dx = col * STEP_X
            dy = row * STEP_Y
            # 再通过二维旋转矩阵修正整体安装角度偏差
            x = int(START_X + dx * cos_rot - dy * sin_rot)
            y = int(START_Y + dx * sin_rot + dy * cos_rot)
            points.append((x, y))

    return points


def validate_points(points):
    # 检查采样点是否全部落在图像范围内
    width = sensor.width()
    height = sensor.height()
    for x, y in points:
        if x < 0 or y < 0 or x >= width or y >= height:
            return False
    return True


def draw_sample_points(img, points):
    # 默认改为画小点而不是十字，减少对预览画面的遮挡
    for row in range(GRID_H):
        for col in range(GRID_W):
            if DRAW_FULL_GRID or (row % DRAW_STRIDE == 0 and col % DRAW_STRIDE == 0):
                x, y = points[row * GRID_W + col]
                if DRAW_POINT_STYLE == "cross":
                    img.draw_cross(x, y, color=(255, 0, 0), size=DRAW_POINT_SIZE)
                else:
                    img.draw_rectangle(x - DRAW_POINT_SIZE,
                                       y - DRAW_POINT_SIZE,
                                       DRAW_POINT_SIZE * 2 + 1,
                                       DRAW_POINT_SIZE * 2 + 1,
                                       color=(255, 0, 0),
                                       fill=True)

    # 额外用绿色标记中间附近的参考点，便于观察整体偏移
    if DRAW_CENTER_MARK:
        center_row = GRID_H // 2
        center_col = GRID_W // 2
        center_x, center_y = points[center_row * GRID_W + center_col]
        img.draw_rectangle(center_x - 2, center_y - 2, 5, 5, color=(0, 255, 0), fill=True)


def rgb_distance_sq(rgb_a, rgb_b):
    # 计算两个 RGB 颜色的平方距离
    # 不开平方，节省运算量，比较大小时效果等价
    dr = rgb_a[0] - rgb_b[0]
    dg = rgb_a[1] - rgb_b[1]
    db = rgb_a[2] - rgb_b[2]
    return dr * dr + dg * dg + db * db


def is_wall_like(r, g, b):
    # 黑灰色墙体的特点是三个通道值比较接近，且整体不会太亮
    avg = (r + g + b) // 3
    spread = abs(r - avg) + abs(g - avg) + abs(b - avg)
    return spread <= WALL_GRAY_SPREAD_MAX and avg <= WALL_GRAY_BRIGHTNESS_MAX


def is_green_like(r, g, b):
    return r <= GREEN_R_MAX and g >= GREEN_G_MIN and b <= GREEN_B_MAX


def is_cyan_like(r, g, b):
    return r <= CYAN_R_MAX and g >= CYAN_G_MIN and b >= CYAN_B_MIN


def is_player_cell(img, x, y):
    green_count = 0
    cyan_count = 0

    for dx, dy in PLAYER_NEIGHBOR_OFFSETS:
        pixel = img.get_pixel(x + dx, y + dy, rgbtuple=True)
        if pixel is None:
            continue
        r, g, b = pixel
        if is_green_like(r, g, b):
            green_count += 1
        if is_cyan_like(r, g, b):
            cyan_count += 1

    return green_count >= 1 and cyan_count >= 1 and (green_count + cyan_count) >= GREEN_CYAN_MIN_COUNT


def classify_pixel_rgb(r, g, b):
    # 先走墙体特判，减少黑灰色被误分到其他鲜艳颜色
    if USE_WALL_SPECIAL_RULE and is_wall_like(r, g, b):
        return CELL_WALL

    # 再和 6 个参考颜色逐个求距离，距离最小者即为分类结果
    best_cell = CELL_EMPTY
    best_distance = 1 << 30
    rgb = (r, g, b)

    for cell_value, ref_rgb in COLOR_REFERENCES.items():
        distance = rgb_distance_sq(rgb, ref_rgb)
        if distance < best_distance:
            best_distance = distance
            best_cell = cell_value

    return best_cell


def classify_pixel_lab(r, g, b):
    # 预留给后续的 LAB 分类逻辑
    # 当前先回退到 RGB 版本，保证主流程先跑通
    return classify_pixel_rgb(r, g, b)


def classify_pixel(r, g, b):
    # 统一分类入口，便于后续切换颜色空间
    if COLOR_MODE == "LAB":
        return classify_pixel_lab(r, g, b)
    return classify_pixel_rgb(r, g, b)


def classify_map(img, points):
    # 对 192 个采样点逐点取色，并同时生成：
    # 1. 二维地图 map_2d，便于按行列理解和调试
    # 2. 一维地图 map_1d，便于按协议直接发送串口
    map_2d = []
    map_1d = []

    for row in range(GRID_H):
        row_data = []
        for col in range(GRID_W):
            x, y = points[row * GRID_W + col]
            # rgbtuple=True 直接返回 (r, g, b)
            pixel = img.get_pixel(x, y, rgbtuple=True)
            if pixel is None:
                # 理论上不应发生；若发生，保守处理为墙体
                cell_value = CELL_WALL
            else:
                r, g, b = pixel
                if is_player_cell(img, x, y):
                    cell_value = CELL_PLAYER
                else:
                    cell_value = classify_pixel(r, g, b)
            row_data.append(cell_value)
            map_1d.append(cell_value)
        map_2d.append(row_data)

    return map_2d, map_1d


def find_player(map_2d):
    # 在分类后的 16x12 地图中查找玩家编码 5
    # 正常情况场上只能有一个玩家
    player_positions = []
    for row in range(GRID_H):
        for col in range(GRID_W):
            if map_2d[row][col] == CELL_PLAYER:
                player_positions.append((col, row))

    if len(player_positions) == 1:
        return player_positions[0], True

    if len(player_positions) > 1:
        return player_positions[0], False

    return (-1, -1), False


def count_cells(map_1d):
    counts = {
        CELL_EMPTY: 0,
        CELL_WALL: 0,
        CELL_BOX: 0,
        CELL_TARGET: 0,
        CELL_BOMB: 0,
        CELL_PLAYER: 0,
    }
    for cell_value in map_1d:
        if cell_value in counts:
            counts[cell_value] += 1
    return counts


def get_player_candidates(map_2d):
    candidates = []
    for row in range(GRID_H):
        for col in range(GRID_W):
            if map_2d[row][col] == CELL_PLAYER:
                candidates.append((col, row))
    return candidates


def get_object_coords(map_2d, cell_value):
    coords = []
    for row in range(GRID_H):
        for col in range(GRID_W):
            if map_2d[row][col] == cell_value:
                coords.append((col, row))
    return coords


def calc_checksum(data):
    # 对指定字节序列做累加和校验，并只保留低 8 位
    checksum = 0
    for value in data:
        checksum = (checksum + value) & 0xFF
    return checksum


def pack_frame(map_1d, player_x, player_y, status):
    # 按约定协议拼出完整 199 字节数据帧
    packet = bytearray(PACKET_LEN)
    packet[0] = FRAME_HEADER
    packet[1] = PAYLOAD_LEN
    packet[2] = status

    # 地图数据从 Byte 3 开始，连续放 192 字节
    for index in range(GRID_W * GRID_H):
        packet[3 + index] = map_1d[index]

    # 玩家坐标和校验
    packet[195] = player_x & 0xFF
    packet[196] = player_y & 0xFF
    packet[197] = calc_checksum(packet[:197])
    packet[198] = FRAME_TAIL
    return packet


def send_frame(uart, packet):
    # 通过串口发送完整数据帧
    uart.write(packet)


def get_status(points_valid, map_1d, player_ok):
    # 当前定义下，只有采样点有效、地图长度正确、且恰好识别到一个玩家时才置为有效
    if not points_valid:
        return STATUS_INVALID
    if len(map_1d) != GRID_W * GRID_H:
        return STATUS_INVALID
    if not player_ok:
        return STATUS_INVALID
    return STATUS_VALID


def draw_status(img, status, player_pos):
    # 在图像左上角绘制当前状态和玩家坐标，便于调试
    status_text = "VALID" if status == STATUS_VALID else "INVALID"
    img.draw_string(2, 2, status_text, color=(255, 255, 255), scale=1)
    img.draw_string(2, 16, "P:%d,%d" % player_pos, color=(255, 255, 255), scale=1)


def print_debug(status, player_pos, fps, map_2d, should_print):
    # 串口终端打印当前帧的状态、玩家位置、帧率和二维地图
    if not DEBUG_PRINT or not should_print:
        return

    print("status=%d player=(%d,%d) fps=%.2f" % (status, player_pos[0], player_pos[1], fps))
    if DEBUG_PRINT_MAP:
        for row in map_2d:
            line = ""
            for cell_value in row:
                line += CELL_CHARS.get(cell_value, "?")
            print(line)


def print_class_counts(map_1d, should_print):
    if not should_print:
        return
    counts = count_cells(map_1d)
    print("counts empty=%d wall=%d box=%d target=%d bomb=%d player=%d" % (
        counts[CELL_EMPTY],
        counts[CELL_WALL],
        counts[CELL_BOX],
        counts[CELL_TARGET],
        counts[CELL_BOMB],
        counts[CELL_PLAYER],
    ))


def print_player_candidates(map_2d, should_print):
    if not should_print:
        return
    candidates = get_player_candidates(map_2d)
    print("player_candidates=%s" % str(candidates))


def print_object_coords(map_2d, should_print):
    if not should_print:
        return
    boxes = get_object_coords(map_2d, CELL_BOX)
    targets = get_object_coords(map_2d, CELL_TARGET)
    players = get_object_coords(map_2d, CELL_PLAYER)
    print("boxes=%s" % str(boxes))
    print("targets=%s" % str(targets))
    print("players=%s" % str(players))


def print_sample_pixels(img, points):
    # 只打印少量关键采样点的坐标与 RGB，避免终端刷爆
    sample_indices = (
        0,
        GRID_W - 1,
        (GRID_H // 2) * GRID_W + (GRID_W // 2),
        (GRID_H - 1) * GRID_W,
        GRID_H * GRID_W - 1,
    )

    for index in sample_indices:
        x, y = points[index]
        pixel = img.get_pixel(x, y, rgbtuple=True)
        print("sample[%d] xy=(%d,%d) rgb=%s" % (index, x, y, str(pixel)))


def print_frame_stats(img):
    # 打印少量全局采样统计，帮助判断是图像本身过曝，还是只是绘制层遮挡
    probe_points = (
        (0, 0),
        (img.width() // 2, 0),
        (img.width() - 1, 0),
        (0, img.height() // 2),
        (img.width() // 2, img.height() // 2),
        (img.width() - 1, img.height() // 2),
        (0, img.height() - 1),
        (img.width() // 2, img.height() - 1),
        (img.width() - 1, img.height() - 1),
    )
    values = []
    for x, y in probe_points:
        pixel = img.get_pixel(x, y, rgbtuple=True)
        values.append(pixel)
    print("frame_probes=%s" % str(values))


def main():
    # 主流程：
    # 1. 初始化摄像头和串口
    # 2. 预先生成固定采样点
    # 3. 循环拍照 -> 取色分类 -> 提取玩家 -> 打包串口发送
    init_sensor()
    # 串口部分暂时注释保留。
    # 当前板端运行报错时，可先关闭 UART，只验证视觉识别主流程。
    # uart = init_uart()
    clock = time.clock()
    points = build_sample_points()
    points_valid = validate_points(points)

    gc.collect()

    while True:
        global debug_frame_count
        debug_frame_count += 1
        should_print = DEBUG_PRINT and (debug_frame_count % DEBUG_PRINT_EVERY == 0)

        clock.tick()
        img = sensor.snapshot()

        map_2d, map_1d = classify_map(img, points)
        player_pos, player_ok = find_player(map_2d)
        status = get_status(points_valid, map_1d, player_ok)

        if DEBUG_PRINT and DEBUG_PRINT_SAMPLES and should_print:
            print_sample_pixels(img, points)
        if DEBUG_PRINT and DEBUG_FRAME_STATS and should_print:
            print_frame_stats(img)
        if DEBUG_PRINT and DEBUG_PRINT_CLASS_COUNTS:
            print_class_counts(map_1d, should_print)
        if DEBUG_PRINT and DEBUG_PRINT_PLAYER_CANDIDATES:
            print_player_candidates(map_2d, should_print)
        if DEBUG_PRINT and DEBUG_PRINT_OBJECT_COORDS:
            print_object_coords(map_2d, should_print)

        if DEBUG_DRAW:
            # 先打印真实采样值，再画调试图层，避免调试图层污染采样结果
            draw_sample_points(img, points)
            if DEBUG_DRAW_STATUS:
                draw_status(img, status, player_pos)

        # 串口发送逻辑暂时注释保留，后续确认 UART API 后再恢复。
        # if status == STATUS_VALID or not SEND_ONLY_WHEN_VALID:
        #     # 当允许发送时，始终打包 199 字节协议发给主控
        #     packet = pack_frame(map_1d, player_pos[0], player_pos[1], status)
        #     send_frame(uart, packet)

        print_debug(status, player_pos, clock.fps(), map_2d, should_print)
        gc.collect()


main()
