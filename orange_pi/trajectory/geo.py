#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geo.py — WGS84 经纬度/高度 <-> 局部 ENU 直角坐标转换模块（gps-imu-fusion 项目）

用途:
  - GPS fix (lat/lon/alt) -> 局部 ENU（原点 = 首个合格 fix，由 set_origin 设定）
  - EKF 融合输出 ENU -> 转回 lat/lon/alt 供 CSV 记录

约定:
  - ENU: n = 北, e = 东, u = 上（右手系，z 向上）；全项目统一 ENU，不混用 NED
  - 原点只可设置一次（set_origin 二次调用返回 False，原点不可变）
  - 小范围等距近似：吊车工作半径 < 500 m 精度足够；不承诺 > 100 km

WGS84 常数:
  - 半长轴 a = 6378137.0 m
  - 扁率 1/f = 1/298.257223563

公式（小范围等距近似，与计划冻结值一致）:
  dN = (lat - lat0) * 111132.95                       [m]
  dE = (lon - lon0) * 111319.49 * cos(lat0_rad)      [m]
  dU = alt - alt0                                     [m]
  逆变换使用与正变换完全相同的常数与基准纬度 lat0，保证往返自洽。
"""

import math
import sys

# ---- WGS84 常数 ----
WGS84_A = 6378137.0             # 半长轴 [m]
WGS84_F = 1.0 / 298.257223563   # 扁率
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)  # 第一偏心率平方

# ---- 小范围等距近似常数（米/度，计划冻结值） ----
M_PER_DEG_LAT = 111132.95       # 纬度 1 度近似弧长 [m/deg]
M_PER_DEG_LON_EQ = 111319.49    # 赤道上经度 1 度近似弧长 [m/deg]（约 a*pi/180）

# ---- 模块级原点状态（只可设置一次） ----
_origin = None  # (lat0_deg, lon0_deg, alt0_m)


def set_origin(lat, lon, alt):
    """设置 ENU 原点（仅可调用一次，原点不可变语义）。

    返回:
      True  - 原点设置成功（首次调用）
      False - 原点已存在，本次调用被拒绝，既有原点保持不变
    """
    global _origin
    if _origin is not None:
        return False
    lat = float(lat)
    cos_lat0 = math.cos(math.radians(lat))
    if abs(cos_lat0) < 1e-12:
        raise ValueError("geo: local ENU frame undefined at poles")
    _origin = (lat, float(lon), float(alt))
    return True


def get_origin():
    """返回当前原点 (lat_deg, lon_deg, alt_m)；未设置时抛 RuntimeError。"""
    if _origin is None:
        raise RuntimeError("geo: origin not set; call set_origin() first")
    return _origin


def latlonalt_to_enu(lat, lon, alt):
    """WGS84 (lat_deg, lon_deg, alt_m) -> 局部 ENU (n, e, u) [m]。"""
    if _origin is None:
        raise RuntimeError("geo: origin not set; call set_origin() first")
    lat0, lon0, alt0 = _origin
    n = (lat - lat0) * M_PER_DEG_LAT
    e = (lon - lon0) * M_PER_DEG_LON_EQ * math.cos(math.radians(lat0))
    u = alt - alt0
    return (n, e, u)


def enu_to_latlonalt(n, e, u):
    """局部 ENU (n, e, u) [m] -> WGS84 (lat_deg, lon_deg, alt_m)。

    使用与正变换完全相同的常数与基准纬度 lat0，保证往返自洽。
    """
    if _origin is None:
        raise RuntimeError("geo: origin not set; call set_origin() first")
    lat0, lon0, alt0 = _origin
    cos_lat0 = math.cos(math.radians(lat0))
    lat = lat0 + n / M_PER_DEG_LAT
    lon = lon0 + e / (M_PER_DEG_LON_EQ * cos_lat0)
    alt = alt0 + u
    return (lat, lon, alt)


# --------------------------------------------------------------------------
# 内置单元测试（python3 geo.py --test）
# --------------------------------------------------------------------------

def _approx(actual, expect, tol_pct):
    """相对容差断言: |actual - expect| <= |expect| * tol_pct/100。"""
    return abs(actual - expect) <= abs(expect) * tol_pct / 100.0


def run_tests():
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(("PASS" if ok else "FAIL") + " - " + name + ((" : " + detail) if detail else ""))

    # -- 测试 1: 原点处 -> (0, 0, 0) --
    ok = set_origin(40.0, 113.0, 100.0) is True
    check("set-origin-first", ok, "set_origin(40,113,100) -> True")
    enu0 = latlonalt_to_enu(40.0, 113.0, 100.0)
    check("origin-enu-zero", all(abs(v) < 1e-9 for v in enu0), "enu(origin) = %r" % (enu0,))

    # -- 测试 2: lat 增 0.001° -> dN ≈ 111.13 m（容差 0.5%） --
    n1, e1, u1 = latlonalt_to_enu(40.0 + 0.001, 113.0, 100.0)
    check("north-vector", _approx(n1, 111.13, 0.5), "dN=%.6f m expect ~111.13 (tol 0.5%%)" % n1)
    check("north-no-cross", abs(e1) < 1e-9 and abs(u1) < 1e-9, "dE=%.3g dU=%.3g" % (e1, u1))
    # 独立参照: WGS84 椭球子午曲率半径 M(lat0) 弧长
    sin2 = math.sin(math.radians(40.0)) ** 2
    m_radius = WGS84_A * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * sin2) ** 1.5
    ref_dn = m_radius * math.radians(0.001)
    check("north-wgs84-ref", _approx(n1, ref_dn, 0.5), "dN=%.6f m expect %.6f (WGS84 M*dlat)" % (n1, ref_dn))

    # -- 测试 3: lon 增 0.001°（lat0=40°）-> dE ≈ 111.31949*cos(40°)*0.001（容差 0.5%） --
    exp_de = M_PER_DEG_LON_EQ * math.cos(math.radians(40.0)) * 0.001
    n2, e2, u2 = latlonalt_to_enu(40.0, 113.0 + 0.001, 100.0)
    check("east-vector", _approx(e2, exp_de, 0.5), "dE=%.6f m expect ~%.6f (tol 0.5%%)" % (e2, exp_de))
    check("east-no-cross", abs(n2) < 1e-9 and abs(u2) < 1e-9, "dN=%.3g dU=%.3g" % (n2, u2))
    # 独立参照: 球面近似 a * dlon_rad * cos(lat0)
    ref_de = WGS84_A * math.radians(0.001) * math.cos(math.radians(40.0))
    check("east-wgs84-ref", _approx(e2, ref_de, 0.5), "dE=%.6f m expect %.6f (a*dlon_rad*cos)" % (e2, ref_de))

    # -- 测试 4: ENU -> WGS84 -> ENU 往返误差 < 1e-6 m --
    worst = 0.0
    pts = [(0.0, 0.0, 0.0), (111.13295, 85.2766, 0.0),
           (300.0, -200.0, 50.0), (-250.0, 180.0, -30.0),
           (499.0, 499.0, 499.0), (-400.0, -350.0, 250.0)]
    for (n, e, u) in pts:
        lat, lon, alt = enu_to_latlonalt(n, e, u)
        n2r, e2r, u2r = latlonalt_to_enu(lat, lon, alt)
        err = max(abs(n - n2r), abs(e - e2r), abs(u - u2r))
        worst = max(worst, err)
    check("roundtrip-enu", worst < 1e-6, "max err = %.3g m (< 1e-6 m)" % worst)

    # -- 测试 5: WGS84 -> ENU -> WGS84 往返（度数级自洽） --
    worst2 = 0.0
    for (lat, lon, alt) in [(40.001, 113.0, 100.0), (40.0, 113.001, 100.5), (40.0, 113.0, 101.0)]:
        n, e, u = latlonalt_to_enu(lat, lon, alt)
        lat2, lon2, alt2 = enu_to_latlonalt(n, e, u)
        worst2 = max(worst2, abs(lat - lat2), abs(lon - lon2), abs(alt - alt2))
    check("roundtrip-wgs84", worst2 < 1e-9, "max err = %.3g deg/m (< 1e-9)" % worst2)

    # -- 测试 6: 原点不可变（二次 set_origin 拒绝且原点不变） --
    ok = set_origin(41.0, 114.0, 200.0) is False
    check("origin-immutable", ok, "second set_origin(41,114,200) -> False")
    lat0, lon0, alt0 = get_origin()
    check("origin-unchanged", lat0 == 40.0 and lon0 == 113.0 and alt0 == 100.0,
          "origin still %r" % (get_origin(),))

    n_fail = sum(1 for _, ok in results if not ok)
    print("-" * 60)
    print("TOTAL: %d tests, %d FAIL" % (len(results), n_fail))
    return 0 if n_fail == 0 else 1


def main(argv):
    if len(argv) >= 2 and argv[1] == "--test":
        return run_tests()
    print(__doc__)
    print("用法: python3 geo.py --test   # 运行内置单元测试")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
