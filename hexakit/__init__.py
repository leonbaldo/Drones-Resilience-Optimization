"""Starter toolkit for the degrading-hexacopter reconfiguration problem.

    from hexakit import io, whitebox
    idx   = io.flight_index()                 # one row per flight in data/flights (all metadata + outcome)
    f     = io.load_flight("F1002")           # dict of 50 Hz arrays + meta
    grid  = io.load_map()                     # occupancy grid, sites
    routes = io.load_routes()                 # route library (A -> B alternatives)
    z     = io.load_health_traces("dbdev_v3") # 1 s health traces z(t) from the trained encoders
"""
from . import io, whitebox  # noqa: F401
