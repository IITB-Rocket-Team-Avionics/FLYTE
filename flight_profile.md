# Flight Profile

## Idle
- Armed, on pad in standby
- should not log data, instead use a running array to detect liftoff

## Liftoff
- Detect launch during sudden acceleration
- Start data logging in flash chip

## Powered Ascent and Coasting (split)
- Keep track of all physical state parameters (velocity, acceleration, altitude, orientation)
- Detect burnout

## Apogee
- Detect apogee and log altitude
- Large omega, altitude maxima logic, low velocity etc.

## Drogue chute(not in small rocket)
- Send pyro charge signal near apogee
- Detect deployment

## Main Chute
- Send pyro charge signal based on decided altitude
- Detect deployment
- Start transferring data from flash chip to sd card, finish before touchdown

## Touchdown
Detect touchdown (and give GPS telemetry)?
