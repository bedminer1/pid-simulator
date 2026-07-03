# PID Simulator

ASCII animated PID controller simulator. Watch a car with random mass and drag chase a target position in real time.

## Usage

```bash
./build.sh
./pid-simulator [Kp Ki Kd] [mass drag]
```

### Examples

```bash
# Random plant, gains = 0 — watch it do nothing
./pid-simulator

# P-only — gets close but never reaches target (steady-state error)
./pid-simulator 1.0 0.0 0.0

# PI — reaches target but may oscillate
./pid-simulator 1.0 0.5 0.0

# PID — fast and stable
./pid-simulator 2.0 0.5 0.3

# Same plant every time (lock mass and drag from a previous run)
./pid-simulator 2.0 0.5 0.3 2.4 0.02
```

## How to read the output

```
 Car: mass=2.4 kg  drag=0.02
 Gains: Kp=2.0  Ki=0.50  Kd=0.30

 Kp=2.00  Ki=0.50  Kd=0.30  |  step   0

 target ──●──  │  actual ──◯──
────────────────┼──────────────────────────────
    200  ◯···················●·················
    190  ◯···················●·················
   ...  ◯ chases ● down the screen
     10  ····················●·················
────────────────┼──────────────────────────────
               │          |                    output

Settled at step 23 (error < 1.0 for 10 steps)
Time to stability: 2.3s
```

- `●` = target position (100)
- `◯` = actual car position
- `╳` = both at same spot
- `|` = current PID output magnitude
- It stops early once the car settles at target

## Building

```bash
./build.sh     # Configure + compile
rm -rf build   # Clean rebuild
```
