"""测试场地, 给 Gazebo 与 2D 评估共用同一份定义。文件与简报里的编号:
    场景 1 = project3 (PPT 第一页, 定义在 worlds/project3.world 与 benchmark_nav.py), 场景 2 = corridor (长墙),
    场景 3 = paper_fig4 (论文), 场景 4 = clutter (柱子)。指令里用的是名称, 不是编号。
    tools/make_world.py   依这里的定义产生 worlds/<名称>.world
    benchmark_nav.py      Gazebo 评测的目标点与障碍物 (第 3 个参数给场地名称)
    rl/eval_compare.py    2D 环境评估
障碍物: ('box', 中心x, 中心y, x方向长, y方向长) / ('cyl', 中心x, 中心y, 半径), 单位 m; 机器狗从原点朝 +x 出发。
要加新场地: 在 SCENES 加一项, 跑 python3 tools/make_world.py, 然后 ./run.sh sim rl world:=/worlds/<名称>.world。
注意起点后方要留 0.8 m 以上 (机器狗静止时会慢慢往后滑), 目标点离障碍物表面至少 0.35 m。"""

ARENA = [('box', 2.9, 2.0, 7.1, 0.1), ('box', 2.9, -2.0, 7.1, 0.1), ('box', -0.6, 0, 0.1, 4.1), ('box', 6.4, 0, 0.1, 4.1)]   # 与 project3 相同的 7 m x 4 m 围墙

SCENES = {
    # 窄通道 + 死巷: 两道交错的长墙 (要蛇行, 与围墙之间的缺口宽 1.2 m)、一道只留 0.6 m 门口的墙、门后一个开口朝门的 U 形死巷,
    # 第 1 段的目标在死巷正后方 (直直走会走进死巷)
    'corridor': dict(
        timeout=240.0,       # 蛇行路线约 10 m, 以实际 0.08 m/s 走要 2 分钟以上
        walls=ARENA,
        obstacles=[
            ('box', 1.3, 0.625, 0.15, 2.75), ('box', 2.7, -0.625, 0.15, 2.75),             # 交错的长墙: 缺口分别在下方、上方
            ('box', 4.1, 1.15, 0.15, 1.7), ('box', 4.1, -1.15, 0.15, 1.7),             # 门口 y -0.3 ~ 0.3
            ('box', 5.3, 0.0, 0.12, 1.42), ('box', 5.05, 0.65, 0.6, 0.12), ('box', 5.05, -0.65, 0.6, 0.12),   # U 形死巷
        ],
        goals=[(5.9, 0.0), (0.0, 0.0), (2.0, 1.5), (3.4, -1.5), (5.9, 1.5), (0.0, -1.5), (5.9, -1.5)]),
    # 密集障碍物: 6 排交错的柱子, 柱间空隙约 0.55 ~ 0.65 m, 没有一条直线可以走
    'clutter': dict(
        timeout=150.0,
        walls=ARENA,
        obstacles=[('cyl', x, y, 0.13) if (i + j) % 3 else ('box', x, y, 0.26, 0.26)
                   for i, x in enumerate([1.0, 1.9, 2.8, 3.7, 4.6, 5.5])
                   for j, y in enumerate([-1.3, -0.4, 0.5, 1.4] if i % 2 == 0 else [-0.85, 0.05, 0.95])],
        goals=[(5.95, 0.5), (0.0, 0.0), (5.95, -1.5), (0.1, 1.5), (3.25, 0.25), (0.1, -1.5), (5.95, 0.5)]),
    # 论文场地: Robust Path Planning for Quadruped Robots in Dynamic and Challenging Environment (IEEE, https://ieeexplore.ieee.org/document/10865160/)
    # 的 Fig. 4: 20 x 20 格的栅格地图, 黑格是静态障碍物 (含 L 形 / U 形陷阱区)。只做静态障碍物, 论文里会动的障碍物没有做。
    # 每格 0.5 m -> 10 m x 10 m; 论文的起点 (19,19) 放在世界原点, 终点在左下角, 世界 (-8.75, -8.75)。见下方 paper_grid()
    'paper_fig4': None,
}

# ---- 论文 Fig. 4 的栅格地图 ----
PAPER_CELL = 0.5                     # 每格边长 (m)。主要路线上最窄处 2 格 = 1.0 m; 有几处 1 格 = 0.5 m 的窄缝 (不在主要路线上)
PAPER_START = (19.5, 19.5)           # 论文起点所在格子的中心 (格子坐标, 图上的刻度 1..21) -> 世界原点
# 黑色障碍物: (x0, x1, y0, y1), 格子坐标, 从图上逐格核对过
PAPER_RECTS = [(1, 12, 20, 21), (10, 12, 15, 20), (12, 15, 15, 17), (13, 15, 12, 15), (15, 17, 14, 15), (19, 21, 16, 18),
               (8, 10, 10, 12), (3, 5, 6, 8), (3, 6, 5, 6), (3, 4, 4, 5), (5, 9, 3, 4), (9, 11, 6, 8),
               (12, 16, 1, 4), (16, 18, 5, 10), (15, 16, 6, 8), (19, 21, 1, 6)]


def paper_grid(gx, gy):
    """格子坐标 -> 世界坐标 (m)。"""
    return (round((gx - PAPER_START[0]) * PAPER_CELL, 3), round((gy - PAPER_START[1]) * PAPER_CELL, 3))


def _paper_box(x0, x1, y0, y1):
    (ax, ay), (bx, by) = paper_grid(x0, y0), paper_grid(x1, y1)
    return ('box', round((ax + bx) / 2, 3), round((ay + by) / 2, 3), round(bx - ax, 3), round(by - ay, 3))


SCENES['paper_fig4'] = dict(
    timeout=400.0,                   # 起点到终点约 17 m, 以实际 0.08 m/s 走要 3.5 分钟以上
    walls=[_paper_box(1, 21, 21, 21.2), _paper_box(1, 21, 0.8, 1), _paper_box(0.8, 1, 0.8, 21.2), _paper_box(21, 21.2, 0.8, 21.2)],
    obstacles=[_paper_box(*r) for r in PAPER_RECTS],
    # 1 论文的终点; 2 回到论文的起点; 3 论文里人工势场法卡住的 L 形陷阱角落; 4 地图中段; 5 再到论文的终点
    # (论文终点在左下角那一格; 格子中心离两面墙只有 0.25 m, 机器狗到不了, 所以往内移半格到 (2, 2), 离墙 0.5 m)
    goals=[paper_grid(2, 2), (0.0, 0.0), paper_grid(16, 16), paper_grid(11.5, 9), paper_grid(2, 2)])
