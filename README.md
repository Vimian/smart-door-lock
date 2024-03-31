# smart-door-lock

This is the semester project for the 2nd semester of the master.

## Circuit

![circuit](./circuit/circuit.svg)

## Build and flash

```
cd src

idf.py build

idf.py -p <PORT> flash
```

## Lights

-   None unlocked and closed
-   Green locked and closed
-   red unlocked and open
-   yellow locked and open
-   yellow/white locked, open and alarm
-   green/cyan locked, closed and alarm
-   ! purple unlocked, open and alarm
-   ! blue alarm

! are states that does not exist. If you see one of these colors, then something broke "PLEASE, FIX!".
