# ZMK Driver for Azoteq IQS5XX Trackpads

A ZMK v0.3 input driver for Azoteq IQS5xx-based trackpads (TPS43 and TPS65 chips). It talks to the pad over I2C, driven by the pad's RDY interrupt, and turns touches into standard ZMK input events — cursor movement, clicks, drags, scrolls, swipes and zoom — so the trackpad works as a mouse on your host, including in split keyboards where the pad sits on a peripheral half.

Note: This driver was originally developed by @stelmakhdigital and I wanted to add some features/bug fixes.  But I don't read Russian, so I asked my AI buddy to make this English [translated](https://github.com/geeksville/zmk_driver_azoteq/commit/4c56195658bbf31851b9892c63f758f50ea48d4d) version.  I hope you find it useful.  It now seems to work well for me.  If you see problems please open issues and I'll do what I can to help.

## Compatibility

This driver should work with any IQS5XX-based trackpad (TPS43 or TPS65).

## Features

- **Trackpad movement** — finger motion is reported as relative cursor movement.
- **Touch reporting** — finger contact is reported as `INPUT_BTN_TOUCH` (1 when one or more fingers are on the pad, 0 when all fingers are lifted).
- **Single tap** — registered as left click.
- **Two-finger tap** — registered as right click.
- **Three-finger tap** — registered as middle click (`INPUT_BTN_2` / `INPUT_BTN_MIDDLE`). When `three-finger-tap` is not enabled, any multi-finger tap reported by the hardware falls back to two-finger-tap (right-click) behavior.
- **Press and hold** — registered as continuous left click, i.e. drag.
- **Two-finger scroll** — modern 'gesture based' two-finger swiping; the dominant axis wins each event so diagonal strokes don't jitter on both axes. Per-axis direction inversion via `invert-scroll-x` / `invert-scroll-y`; speed via `scroll-sensitivity`.
- **1-finger and 3-finger directional swipes** — send `INPUT_BTN_WEST`/`EAST`/`NORTH`/`SOUTH` events; map them to any keys/actions in your `.overlay` (either or both can be enabled: `swipes` for 1-finger, `three-finger-swipe` for 3-finger only, independently of each other). Three-finger swipe actuations are throttled by `three-finger-swipe-throttle-ms` (default 300 ms, 0 disables) so one continuous motion doesn't fire repeatedly.
- **Pinch-to-zoom** — reported as movement on the `INPUT_REL_MISC` 'wheel'; add dts rules to map this to something your apps understand.
- **Edge drag continuation** — optional lift-and-reposition support at trackpad boundaries: during a drag, lifting a finger inside the edge buffer (`drag-continuation-edge-buffer`) keeps the button held for up to `drag-continuation-timeout-ms` so you can reposition and keep dragging.
- **Drag-continuation auto-slide** (new) — with `drag-continuation-slide` (requires `drag-continuation`): while dragging with one finger resting nearly still inside the edge buffer, after a dwell of `drag-continuation-slide-dwell-ms` (default 250 ms) the driver injects constant-speed cursor motion away from the trackpad center. A finger counts as still when its per-report absolute movement is at most `drag-continuation-slide-still-px` (default 12). Speed is `drag-continuation-slide-speed` trackpad px/s (default 400, constant, no ramp; minimum effective value 100 px/s). Direction is the vector from pad center toward the resting finger, re-derived every tick, so moving the resting finger within the buffer steers instantly. Lifting the finger falls through to the normal lift-and-reposition continuation — the button stays held for the timeout as usual.
- **Power management** — integrates with ZMK's activity-state system: the pad sleeps when ZMK sleeps, and also when ZMK is idle if `idle-sleep` is set. Chip-level low-power cadence knobs (`report-rate-lp1/lp2`, `timeout-*`) exist for deeper tuning; see the binding YAML.
- **Runtime-reset self-heal** — the driver detects the chip's `SHOW_RESET` flag in the normal event loop (ESD, watchdog, brownout) and automatically re-acknowledges and re-applies its full configuration.
- **Split-keyboard support** — via ZMK's `zmk,input-split`, the pad can live on a peripheral half while a listener on the central dispatches its events.

## Quick start

- In the configuration file (for the appropriate half), enable the driver:

```
CONFIG_INPUT_TPS43=y
```

- In the `.overlay` file, specify `compatible = "azoteq,tps43"` inside the i2c node where the trackpad will be used:

```
&i2c0 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;
    pinctrl-0 = <&i2c0_default>;  /* Configuration for SDA and SCL */
    pinctrl-1 = <&i2c0_sleep>;    /* Configuration for SDA and SCL */
    pinctrl-names = "default", "sleep";

    tps43_trackpad: trackpad@74 {
        compatible = "azoteq,tps43";
        reg = <0x74>;
        status = "okay";
        
        /* GPIO connections */
        rdy-gpios = <&pro_micro 21 GPIO_ACTIVE_HIGH>;  /* RDY pin */
        rst-gpios = <&pro_micro 20 GPIO_ACTIVE_HIGH>;  /* RST pin */

        enable-power-management;
        
        sensitivity = <100>;           /* 100% = normal state */
        scroll-sensitivity = <50>;     /* 50% = normal state */
        zoom-sensitivity = <50>;       /* 50% = normal state */

        filter-settings=<0x0B>;        /* See filter description in `available settings` */
        // filter-dynamic-bottom=<7>;     /* Dynamic filter bottom beta (optional) */
        // filter-dynamic-lower=<6>;      /* Dynamic filter lower speed (optional) */
        // filter-dynamic-upper=<0xFA>;   /* Dynamic filter upper speed (optional) */
        // x-resolution=<1024>;           /* X resolution in pixels (optional, default: 1024) */
        // y-resolution=<1024>;           /* Y resolution in pixels (optional, default: 1024) */

        // swipe-initial-distance=<500>;          /* Swipe initial distance in px (optional) */
        // swipe-initial-time=<200>;              /* Swipe initial time in ms (optional) */
        // swipe-angle=<20>;                      /* Max swipe angle in degrees (optional) */
        // swipe-consecutive-distance=<200>;      /* Swipe consecutive distance in px (optional) */
        // swipe-consecutive-time=<150>;          /* Swipe consecutive time in ms (optional) */
        // scroll-initial-distance=<100>;         /* Scroll initial distance in px (optional) */
        // scroll-angle=<30>;                     /* Max scroll angle in degrees (optional) */
        // zoom-initial-distance=<100>;           /* Zoom initial distance in px (optional) */
        // zoom-consecutive-distance=<50>;        /* Zoom consecutive distance in px (optional) */

        scroll;
        two-finger-tap;
        // three-finger-tap;                    /* 3-finger tap (middle click INPUT_BTN_2) (optional) */
        single-tap;
        press-and-hold;
        swipes;
        // three-finger-swipe;                    /* 3-finger swipe only, without enabling 1-finger swipe (optional) */
        // three-finger-swipe-throttle-ms=<300>;  /* Min ms between 3-finger swipe actuations, 0 disables (optional, default 300) */
        zoom;

        // drag-continuation;                        /* Keep drag alive when lifting in the edge buffer (optional) */
        // drag-continuation-timeout-ms=<600>;       /* Lift budget for repositioning (optional, default 600) */
        // drag-continuation-edge-buffer=<80>;       /* Edge trigger zone width in px (optional, default 80) */
        // drag-continuation-slide;                  /* Auto-slide while a finger rests in the edge buffer during a drag (optional, requires drag-continuation) */
        // drag-continuation-slide-dwell-ms=<250>;   /* Still-in-buffer time before sliding starts (optional, default 250) */
        // drag-continuation-slide-still-px=<12>;    /* Per-report movement at or below which the finger counts as still (optional, default 12) */
        // drag-continuation-slide-speed=<400>;      /* Slide speed in trackpad px/s, constant; min effective 100 (optional, default 400) */

        switch-xy;
        invert-scroll-y;
    };
};
```

## Configuration reference

User-facing behavioral properties (all optional; boolean properties default to off, int properties to the listed value):

| Property | Type | Default | Meaning |
|---|---|---|---|
| `single-tap` | boolean | off | Single tap = left click. |
| `two-finger-tap` | boolean | off | Two-finger tap = right click. |
| `three-finger-tap` | boolean | off | Three-finger tap = middle click (`INPUT_BTN_2`); without it, multi-finger taps fall back to two-finger-tap behavior. |
| `press-and-hold` | boolean | off | Hold = continuous left click (drag). |
| `scroll` | boolean | off | Two-finger dominant-axis scrolling. |
| `zoom` | boolean | off | Pinch-to-zoom, emitted on `INPUT_REL_MISC`. |
| `swipes` | boolean | off | 1-finger directional swipes → `INPUT_BTN_NORTH/SOUTH/EAST/WEST`. |
| `three-finger-swipe` | boolean | off | 3-finger-only swipes → `INPUT_BTN_NORTH/SOUTH/EAST/WEST`; independent of `swipes`. |
| `three-finger-swipe-throttle-ms` | int | 300 | Minimum ms between consecutive 3-finger swipe actuations; 0 disables. |
| `drag-continuation` | boolean | off | Edge lift-and-reposition drag continuation. |
| `drag-continuation-timeout-ms` | int | 600 | Maximum lift time (ms) allowed to reposition during an edge drag. |
| `drag-continuation-edge-buffer` | int | 80 | Border width in trackpad px defining the edge trigger zone (~8% of the pad at 1024x1024). |
| `drag-continuation-slide` | boolean | off | Auto-slide away from pad center while a finger rests in the edge buffer during a drag. Requires `drag-continuation`; lift falls through to normal continuation. |
| `drag-continuation-slide-dwell-ms` | int | 250 | Still-in-buffer hold time before sliding starts; movement above the still threshold restarts it. |
| `drag-continuation-slide-still-px` | int | 12 | Per-report absolute movement (sum of \|dx\|+\|dy\|) at or below which the finger counts as still. |
| `drag-continuation-slide-speed` | int | 400 | Slide speed in trackpad px/s, constant, no ramp. Minimum effective value 100 px/s. |
| `sensitivity` | int | 100 | Touch sensitivity percentage (100 = normal, higher = more sensitive). |
| `scroll-sensitivity` | int | 100 | Scroll sensitivity percentage. |
| `zoom-sensitivity` | int | 100 | Zoom sensitivity percentage. |
| `switch-xy` | boolean | off | Swap X and Y axes. |
| `invert-x` | boolean | off | Invert X coordinate values. |
| `invert-y` | boolean | off | Invert Y coordinate values. |
| `invert-scroll-x` | boolean | off | Invert X scroll values. |
| `invert-scroll-y` | boolean | off | Invert Y scroll values. |
| `enable-power-management` | boolean | on | Automatic suspend/resume power management (driver default: enabled). |
| `idle-sleep` | boolean | off | Also sleep the trackpad while ZMK is in the IDLE activity state (by default only SLEEP sleeps it). |

Register-level tuning properties (`swipe-initial-*`, `ati-*`, `report-rate-*`, `filter-*`, `timeout-*`, `tap-*`, `hold-time` and friends) keep their descriptions in the binding file: [available trackpad settings](./dts/bindings/input/azoteq,tps43-common.yaml).

## Listener / split setup

Now configure a listener for tracking touches:

> Important: If your trackpad `.overlay` configuration is on the central device, use `Option 1`. If the trackpad is located on the peripheral half of the keyboard, use `Option 2`.

**Option 1 — central listener**

Simply specify the listener on the central device itself.

```
/ {
    tps43_input: tps43_input {
        compatible = "zmk,input-listener";
        device = <&tps43_trackpad>;
    };
};
```

**Option 2 — peripheral trackpad via `zmk,input-split`**

Configure `split_inputs` where the trackpad is used (on the peripheral half):

```
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        tps43_split: tps43_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
            device = <&tps43_trackpad>;
        };
    };
};
```

On the central part, declare the matching proxy (no `device` property — the device lives on the peripheral side) plus a listener pointed at it:

```
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        tps43_split: tps43_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
            /* No device property here - this is a proxy on the central side */
        };
    };

    tps43_listener: tps43_listener {
        compatible = "zmk,input-listener";
        device = <&tps43_split>;
        status = "okay";
    };
};
```

## Wiring

> Configuring the Azoteq trackpad requires 5 pins!

Power:
3V on nice!nano -> VDD on IQS5xx.
GND (Ground) on nice!nano -> GND on IQS5xx.

I2C signals:
SDA on nice!nano -> SDA on IQS5xx.
SCL on nice!nano -> SCL on IQS5xx.

"DR" or "RDY" pin on IQS5xx -> Any available GPIO on nice!nano. In the devicetree, this pin is specified as rdy-gpios.
"RST" pin is used to initialize device reset. In the devicetree, this pin is specified as rst-gpios.

## Maintainers note

On boot (and after any chip-detected reset), the driver holds RST, waits for firmware load, waits for the `SHOW_RESET` flag, sends `ACK_RESET`, writes the full configuration, and only then arms the RDY interrupt. The runtime work loop re-detects `SHOW_RESET` and reconfigures, self-healing from ESD/watchdog events. Robustness details — priority-inheriting `k_mutex`, immediate I2C window close, RDY drain on boot/resume, zero-delta suppression — are in the comments in [drivers/input/tps43.c](./drivers/input/tps43.c); see also [doc/develop.md](./doc/develop.md).
