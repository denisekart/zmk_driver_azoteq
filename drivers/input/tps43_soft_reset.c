/*
 * Global soft-reset behavior for the Azoteq TPS43 trackpad setup.
 *
 * Bound via a leader sequence (e.g. <leader> Z Z Z), the behavior restores
 * the input stack to a power-on-fresh state in one press. As a
 * BEHAVIOR_LOCALITY_GLOBAL binding it is invoked locally on the central AND
 * forwarded to every peripheral before that, so each firmware runs its
 * half of the recovery:
 *  - central (or non-split): releases all stuck keymap positions, clears and
 *    sends the keyboard/consumer/mouse HID reports, cancels a pending
 *    leader-key sequence, stops the mouse mover and drops all non-default
 *    layers back to the default layer;
 *  - peripheral owning the TPS43: re-initializes the chip (hardware RST
 *    pulse + full re-configuration + RDY IRQ re-arm) asynchronously via
 *    tps43_schedule_reset().
 *
 * The keymap, endpoints and leader-key modules only exist on the central
 * half of a split build, hence the compile-time role split below.
 */

#define DT_DRV_COMPAT zmk_behavior_tps43_soft_reset

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

/* Prototype for tps43_schedule_reset(); the chip node itself only exists in
 * the devicetree of the half that wires it up, guarded per branch below. */
#include "tps43.h"

LOG_MODULE_REGISTER(tps43_soft_reset, CONFIG_ZMK_LOG_LEVEL);

/* Expands to the async chip re-init iff an enabled azoteq,tps43 node exists
 * in THIS firmware's devicetree (right overlay only on a split build; the
 * driver object is not even linked where INPUT_TPS43 is off). */
#define TPS43_SOFT_RESET_MaybeResetChip()                                                          \
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(tps43_trackpad), okay),                             \
               ((void)tps43_schedule_reset(DEVICE_DT_GET(DT_NODELABEL(tps43_trackpad))));)

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_LEADER_KEY)
extern void deactivate_leader_key(void); /* leader-key module, non-static */
#endif

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/matrix.h>
#include <zmk/events/position_state_changed.h>

static int soft_reset_on_binding_pressed(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    LOG_INF("TPS43 soft reset invoked");

    /* release stuck positions, flush HID */
    for (uint32_t p = 0; p < ZMK_KEYMAP_LEN; p++) {
        zmk_keymap_position_state_changed(ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL, p, false,
                                          k_uptime_get_32());
    }
    zmk_endpoints_clear_current(); /* clears kbd+consumer+mouse, sends reports */

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_LEADER_KEY)
    deactivate_leader_key();
#endif

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_MOUSE_MOVER)
    extern void zmk_mouse_mover_force_off(void); /* zmk-mouse-mover, central-only */
    zmk_mouse_mover_force_off();
#endif

    zmk_keymap_layers_state_t st = zmk_keymap_layer_state();
    for (zmk_keymap_layer_index_t idx = 0; idx < ZMK_KEYMAP_LAYERS_LEN; idx++) {
        zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(idx);
        if (id != ZMK_KEYMAP_LAYER_ID_INVAL && id != zmk_keymap_layer_default() &&
            (st & BIT(id))) {
            zmk_keymap_layer_toggle(id);
        }
    }
    zmk_keymap_layer_to(zmk_keymap_layer_default()); /* default layer selected */

    /* Non-split builds carry the chip here as well and reset it directly. */
    TPS43_SOFT_RESET_MaybeResetChip();

    return ZMK_BEHAVIOR_OPAQUE;
}

#else /* split peripheral: where the TPS43 chip lives */

static int soft_reset_on_binding_pressed(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    LOG_INF("TPS43 soft reset invoked");

    /* Positions, HID reports and layers are all owned by the central, which
     * runs its half of the recovery from its own local invocation. */
    TPS43_SOFT_RESET_MaybeResetChip();

    return ZMK_BEHAVIOR_OPAQUE;
}

#endif /* role split */

static int soft_reset_on_binding_released(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api tps43_soft_reset_driver_api = {
    .binding_pressed = soft_reset_on_binding_pressed,
    .binding_released = soft_reset_on_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define TPS43_SOFT_RESET_INST(n)                                                                   \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &tps43_soft_reset_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TPS43_SOFT_RESET_INST)
