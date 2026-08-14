"""CLI for the Aurora simulator.  py -m aurora_sim --port COM4 -v"""

from __future__ import annotations

import argparse
import os
import sys

from .dynamics import Dynamics
from .link import SerialLink
from .model import AuroraModel
from .rtu import RtuSlave

HERE = os.path.dirname(os.path.abspath(__file__))
SNAPSHOT = os.path.join(HERE, os.pardir, "sample_snapshot.json")


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="aurora_sim",
        description="Simulate the WaterFurnace Aurora ABC over RS-485 Modbus-RTU.")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM4")
    ap.add_argument("--addr", type=int, default=1, help="slave address (default 1)")
    ap.add_argument("--snapshot", default=SNAPSHOT, help="captured register JSON")
    ap.add_argument("--strict", action="store_true",
                    help="exception 2 for registers absent from the snapshot")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every frame")

    beh = ap.add_argument_group("behaviour")
    beh.add_argument("--apply-delay", type=float, default=12.0, metavar="S",
                     help="IZ2 setpoint sync delay in seconds (real unit: 10-20)")
    beh.add_argument("--reassert-after", type=float, default=None, metavar="S",
                     help="seconds until the IZ2 schedule reasserts its own setpoint "
                          "(off by default; the real cadence was never measured)")
    beh.add_argument("--scenario", default="steady", choices=Dynamics.SCENARIOS,
                     help="running behaviour: steady (captured values + noise), "
                          "cooling, heating, or fp1-freeze (drives the chronic E5 "
                          "source-side freeze to lockout)")
    beh.add_argument("--time-scale", type=float, default=1.0, metavar="X",
                     help="run the physics X times faster than real time "
                          "(e.g. 60 to watch an hour pass in a minute)")
    beh.add_argument("--fault", type=int, default=None, metavar="ECODE",
                     help="inject a stored fault E-code into reg 25")
    beh.add_argument("--lockout", action="store_true",
                     help="set the lockout bit (reg 25 bit 15) with --fault")

    imp = ap.add_argument_group("impairments (bus realism)")
    imp.add_argument("--latency-ms", type=float, default=0.0,
                     help="extra response latency")
    imp.add_argument("--drop-rate", type=float, default=0.0, metavar="P",
                     help="probability of dropping a response entirely")
    imp.add_argument("--corrupt-rate", type=float, default=0.0, metavar="P",
                     help="probability of flipping one bit in a response")
    imp.add_argument("--stats-every", type=float, default=0.0, metavar="S",
                     help="print a statistics line this often")

    args = ap.parse_args(argv)

    model = AuroraModel(args.snapshot,
                        apply_delay_s=args.apply_delay,
                        reassert_after_s=args.reassert_after)
    model.attach_dynamics(Dynamics(model, scenario=args.scenario,
                                   time_scale=args.time_scale))
    if args.fault is not None:
        model.inject_fault(args.fault, args.lockout)

    slave = RtuSlave(model, addr=args.addr, strict=args.strict)
    link = SerialLink(args.port, slave, verbose=args.verbose,
                      impair={"latency_ms": args.latency_ms,
                              "drop_rate": args.drop_rate,
                              "corrupt_rate": args.corrupt_rate},
                      stats_every=args.stats_every)

    print(f"Aurora simulator on {args.port} @ 19200 8E1, slave addr {args.addr}")
    print(f"  {len(model.regs)} registers, snapshot captured {model.captured_at}")
    print(f"  {len(model.zones)} IZ2 zones: " + ", ".join(
        f"Z{z.number} {z.temp_f:.1f}F set {z.heat_sp}/{z.cool_sp}" for z in model.zones))
    print(f"  scenario: {args.scenario}"
          + (f" at {args.time_scale}x real time" if args.time_scale != 1.0 else ""))
    print(f"  setpoint apply delay {args.apply_delay}s"
          + (f", schedule reasserts after {args.reassert_after}s"
             if args.reassert_after else ", schedule reassert off"))
    code, lock = model.current_fault()
    print(f"  fault reg 25: E{code}" + (" LOCKOUT" if lock else "") if code else "  fault reg 25: clear")
    if any([args.drop_rate, args.corrupt_rate, args.latency_ms]):
        print(f"  impairments: latency={args.latency_ms}ms drop={args.drop_rate} "
              f"corrupt={args.corrupt_rate}")
    print("  waiting for master... (Ctrl-C to stop)\n")

    try:
        link.run()
    except KeyboardInterrupt:
        print("\n" + link.stats.line())
        print(f"[model] {model.stats}")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
