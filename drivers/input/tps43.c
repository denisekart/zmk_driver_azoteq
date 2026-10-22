#include <stdint.h>
#define DT_DRV_COMPAT azoteq_tps43

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "tps43.h"

LOG_MODULE_REGISTER(tps43, CONFIG_INPUT_LOG_LEVEL);

static int tps43_configure_device(const struct device *dev);
static int tps43_full_reinit(const struct device *dev);

/**
 * @brief Ends communication window with trackpad
 *
 * After each read of trackpad registers, it is necessary to end the communication window
 * by writing the special address 0xEEEE, which causes a NACK from the device.
 * This is a mandatory step according to the IQS5xx protocol.
 *
 * @param dev Pointer to trackpad device
 */
static void tps43_end_communication_window(const struct device *dev) {
    const struct tps43_config *config = dev->config;
    uint8_t end_buf[2] = {TPS43_REG_END_COMM_WINDOW >> 8, TPS43_REG_END_COMM_WINDOW & 0xFF};

    int ret = i2c_write_dt(&config->i2c_bus, end_buf, sizeof(end_buf));
    if (ret != 0 && ret != -EIO) {
        LOG_DBG("End communication window write returned: %d (NACK expected)", ret);
    }
}

/**
 * @brief Reads a sequence of trackpad registers
 *
 * Performs reading of multiple bytes from sequential trackpad registers,
 * starting from the specified address. Used for reading related registers,
 * such as gesture events (GESTURE_EVENTS_0 and GESTURE_EVENTS_1).
 *
 * @param dev Pointer to trackpad device
 * @param reg Starting register address (16-bit)
 * @param val Pointer to buffer for data
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
static int read_sequence_registers(const struct device *dev, uint16_t reg, void *val, size_t len) {
    const struct tps43_config *config = dev->config;
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(reg & 0xFF);

    return i2c_write_read_dt(&config->i2c_bus, addr_buf, 2, val, len);
}

/**
 * @brief Reads a 16-bit trackpad register via I2C
 *
 * Performs reading of a 16-bit value from the specified trackpad register.
 * Data is interpreted as big-endian (MSB first).
 *
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Pointer to variable to store the read value
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[2];
    // forms 2-byte register address: (MSB, LSB)
    // MSB: shift right by 8 bits (0x2F00 -> 0x2F)
    // LSB: bitwise AND with mask - mask leaves only lower byte (0x2F00 -> 0x00)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;

    // writes register address (reg_buf) and reads 2 bytes of data (into buffer buf)
    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x read error: %d", reg, ret);
        return ret;
    }

    // converts big-endian data (MSB first) back to 16-bit value
    *val = (buf[0] << 8) | buf[1];
    return 0;
}

/**
 * @brief Writes a 16-bit value to trackpad register via I2C
 *
 * Performs writing of a 16-bit value to the specified trackpad register.
 * Data is transmitted as big-endian (MSB first).
 *
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Value to write (16-bit)
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_write_reg16(const struct device *dev, uint16_t reg, uint16_t val)
{
    const struct tps43_config *config = dev->config;
    // forms 4-byte register address: (MSB, LSB, MSB_VALUE, LSB_VALUE)
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};
    int ret;

    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x write error: %d", reg, ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Reads an 8-bit trackpad register via I2C
 *
 * Performs reading of an 8-bit value from the specified trackpad register.
 * Used for reading most configuration and status registers.
 *
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Pointer to variable to store the read value
 * @param with_err Flag to log error or expected behavior
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_read_reg8_w_err(const struct device *dev, uint16_t reg, uint8_t *val, bool with_err)
{
    const struct tps43_config *config = dev->config;
    // forms 2-byte register address: (MSB, LSB)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;

    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), val, 1);
    if (ret != 0) {
        if (!with_err) {
            LOG_DBG("Expected completion of register 0x%04x read: %d", reg, ret);
        } else {
            LOG_ERR("Register 0x%04x read error: %d", reg, ret);
        }
        return ret;
    }
    return ret;
}

static inline int tps43_i2c_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val)
{
    return tps43_i2c_read_reg8_w_err(dev, reg, val, true);
}

/**
 * @brief Writes an 8-bit value to trackpad register via I2C
 *
 * Performs writing of an 8-bit value to the specified trackpad register.
 * Used for writing configuration and control registers.
 *
 * @param dev Pointer to trackpad device
 * @param reg Register address (16-bit)
 * @param val Value to write (8-bit)
 * @return 0 on success, negative error code on failure
 */
static int tps43_i2c_write_reg8(const struct device *dev, uint16_t reg, uint8_t val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};
    int ret;

    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Register 0x%04x write error: %d", reg, ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Callback handler for RDY pin interrupt from trackpad
 *
 * Called when the RDY (Ready) pin state of the trackpad changes,
 * signaling that new data is available for reading.
 * Schedules execution of work handler to read the data.
 *
 * @param dev Pointer to trackpad device
 * @param cb Pointer to GPIO callback structure
 * @param pins Mask of pins that triggered the interrupt
 */
 static void tps43_rdy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
     struct tps43_drv_data *drv_data = CONTAINER_OF(cb, struct tps43_drv_data, rdy_cb);

     k_work_submit(&drv_data->work);
 }

/** Force communication start with the trackpad.  See datasheet 8.8.2 */
static void tps43_force_communication(const struct device *dev) {
    // Do a bogus read where we don't care about a possible NACK
    uint8_t control_reg = 0;
    tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg, false);
}

/* TOUCH/BTN_0 releases are one BLE split packet away from being lost during
 * wake/link re-establishment; a lost release pins the layer keyed on INPUT_BTN_TOUCH
 * or leaves a stuck left-click on the host. Re-emit shortly after every
 * synthesized release that ends a session or drag. */
#define TPS43_TOUCH_RELEASE_RETRY_MS 250

static void tps43_touch_release_retry(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data =
        CONTAINER_OF(dwork, struct tps43_drv_data, touch_release_work);

    /* Skip if state moved on since: a stray 0 mid-touch would drop the layer
     * with no 1->0 edge to re-assert it, and a stray 0 while a drag owner
     * still holds BTN_0 would cancel a live drag. */
    if (!drv_data->touching) {
        input_report_key(drv_data->dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    }
    if (!drv_data->drag_active && !drv_data->drag_continuation_active) {
        input_report_key(drv_data->dev, INPUT_BTN_0, 0, true, K_FOREVER);
    }
}

static void tps43_release_touch(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;

    drv_data->touching = false;
    input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    k_work_reschedule(&drv_data->touch_release_work, K_MSEC(TPS43_TOUCH_RELEASE_RETRY_MS));
}

/* Single exit funnel for the edge drag-continuation auto-slide.
 * Caller must hold drv_data->lock. */
static inline void tps43_slide_stop(struct tps43_drv_data *drv_data) {
    k_work_cancel_delayable(&drv_data->slide_work);
    drv_data->slide_pending = false;
    drv_data->slide_active = false;
}

/* Tear down the touch session on I2C failure or chip reset: a dangling
 * TOUCH/BTN_0 release lost on the BLE hop pins the central's layer. */
static void tps43_teardown_session(struct tps43_drv_data *drv_data) {
    const struct device *dev = drv_data->dev;

    /* Auto-slide is drag-scoped: any session teardown ends it. */
    tps43_slide_stop(drv_data);

    if (drv_data->touching) {
        tps43_release_touch(dev);
    }
    if (drv_data->drag_button_pressed || drv_data->drag_active ||
        drv_data->drag_continuation_active) {
        k_work_cancel_delayable(&drv_data->drag_continuation_work);
        drv_data->drag_continuation_active = false;
        drv_data->drag_active = false;
        drv_data->drag_button_pressed = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
        k_work_reschedule(&drv_data->touch_release_work, K_MSEC(TPS43_TOUCH_RELEASE_RETRY_MS));
    }
    drv_data->touch_session_fingers = 0;
    drv_data->prev_num_fingers = 0;
}

/**
 * @brief Internal function to put trackpad into suspend/resume mode
 *
 * Controls the SYSTEM_CONTROL_1 register (0x0432), setting or clearing the SUSPEND bit.
 * In suspend mode, the trackpad enters a low power consumption state and does not process
 * touches until wake-up.
 *
 * @param dev Pointer to trackpad device
 * @param suspend true - enter suspend, false - exit suspend
 * @param lock_held true if mutex is already held (for internal use)
 * @return 0 on success, negative error code on failure
 */
static int tps43_set_suspend_internal(const struct device *dev, bool suspend, bool lock_held) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret = 0;

    // If power management is disabled, device not initialized, or device is already in the desired state, do nothing
    if (!drv_data->initialized || drv_data->suspended == suspend || !config->enable_power_management) {
        return 0;
    }

    // Acquire mutex if not already held
    if (!lock_held) {
        int err = -EBUSY;
        for (int attempt = 0; attempt < 3; attempt++) {
            err = k_mutex_lock(&drv_data->lock, K_MSEC(50));
            if (err == 0) {
                break;
            }
        }
        if (err != 0) {
            LOG_WRN("Failed to acquire mutex for suspend/resume");
            return -EBUSY;
        }
    }

    /* Synthesized releases run under the lock: a lockless read of the gate
     * flags can sample a handler mid-assert (TOUCH/BTN_0 1 emitted, its
     * end-of-handler flag update pending) and strand that press across the
     * sleep. A failed lock aborts the suspend above, so no release is owed
     * on that leg. */
    if (suspend) {
        if (drv_data->touching) {
            tps43_release_touch(dev);
        }

        tps43_slide_stop(drv_data);

        if (drv_data->drag_active || drv_data->drag_continuation_active ||
            drv_data->drag_button_pressed) {
            k_work_cancel_delayable(&drv_data->drag_continuation_work);
            drv_data->drag_continuation_active = false;
            drv_data->drag_active = false;
            if (drv_data->drag_button_pressed) {
                drv_data->drag_button_pressed = false;
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
                k_work_reschedule(&drv_data->touch_release_work,
                                  K_MSEC(TPS43_TOUCH_RELEASE_RETRY_MS));
            }
        }
    }

    // Disable RDY interrupts when entering suspend (before any I2C operations)
    // This prevents race condition when RDY fires between suspend attempt and flag setting
    if (suspend && config->rdy_gpio.port != NULL) {
        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
        if (ret == 0) {
            LOG_INF("RDY interrupts disabled before suspend");
        }
    }

    uint8_t control_reg = 0;

    // When exiting suspend, first transaction will return NACK (section 7.3.1)
    if (drv_data->suspended && !suspend) {
        tps43_force_communication(dev);
        k_sleep(K_MSEC(1)); // need at least 200uS before 2nd read
        LOG_INF("I2C Wake: device awakened from suspend");

        // After wake-up, read register again
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg);

    } else if (!drv_data->suspended) {
        // Read current value
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg);
    }

    if (suspend) {
        control_reg |= TPS43_SUSPEND;
        LOG_INF("Entering suspend (low power consumption)");
    } else {
        control_reg &= ~TPS43_SUSPEND;
        LOG_INF("Exiting suspend");
    }

    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, control_reg);
    if (ret != 0) {
        if (ret == -EIO && suspend) {
            LOG_INF("Failed to write suspend, device already in suspend");
            drv_data->suspended = true;
            ret = 0;
            goto done;
        }
        LOG_ERR("SYSTEM_CONTROL_1 write error: %d", ret);
        goto done;
    }

    drv_data->suspended = suspend;

done:
    /* Re-enable after resume, and after a *failed* suspend too: a failed
     * suspend leaves suspended==false with the IRQ masked, and a pad awake
     * but IRQ-deaf stays dead until the next full cycle. */
    if ((!suspend || ret != 0) && config->rdy_gpio.port != NULL) {
        int gpio_ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (gpio_ret == 0) {
            LOG_INF("RDY interrupts enabled");
            if (gpio_pin_get_dt(&config->rdy_gpio) > 0) {
                k_work_submit(&drv_data->work);
            }
        } else {
            LOG_ERR("Failed to enable RDY interrupt: %d", gpio_ret);
            if (ret == 0) {
                ret = gpio_ret;
            }
        }
    }
    if (!suspend && ret == 0 && drv_data->needs_reconfigure) {
        /* Deferred reconfigure from a reset-during-suspend; settle time has
         * elapsed while asleep, so re-run configuration inline under lock. */
        drv_data->needs_reconfigure = false;
        tps43_configure_device(dev);
    }
    tps43_end_communication_window(dev);
    if (!lock_held) {
        k_mutex_unlock(&drv_data->lock);
    }
    return ret;
}

/** Common swipe handling for 1 and 3 finger variants */
static void tps43_handle_swipe(const struct device *dev, uint8_t num_fingers, int16_t rel_x, int16_t rel_y) {
    const struct tps43_config *config = dev->config;
    struct tps43_drv_data *drv_data = dev->data;

    bool enabled = false;

    if (num_fingers == 3) {
        if (config->swipes || config->three_finger_swipe) {
            enabled = true;
        }
    }

    if (num_fingers == 1) {
        if (config->swipes) {
            enabled = true;
        }
    }

    if (!enabled) {
        return;
    }

    // Three-finger swipe is derived from raw movement deltas on every report while
    // 3 fingers are down, rather than a single hardware-gated gesture event like the
    // 1-finger swipe. Throttle it so one continuous swipe motion doesn't fire repeatedly.
    if (num_fingers == 3 && config->three_finger_swipe_throttle_ms > 0) {
        int64_t now = k_uptime_get();
        if ((now - drv_data->last_three_finger_swipe_ms) < config->three_finger_swipe_throttle_ms) {
            return;
        }
        drv_data->last_three_finger_swipe_ms = now;
    }

    if (rel_x < 0) {
        LOG_DBG("%d-finger swipe left - INPUT_BTN_WEST", num_fingers);
        input_report_key(dev, INPUT_BTN_WEST, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_WEST, 0, true, K_FOREVER);
    }
    if (rel_x > 0) {
        LOG_DBG("%d-finger swipe right - INPUT_BTN_EAST", num_fingers);
        input_report_key(dev, INPUT_BTN_EAST, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_EAST, 0, true, K_FOREVER);
    }
    if (rel_y < 0) {
        LOG_DBG("%d-finger swipe up - INPUT_BTN_NORTH", num_fingers);
        input_report_key(dev, INPUT_BTN_NORTH, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_NORTH, 0, true, K_FOREVER);
    }
    if (rel_y > 0) {
        LOG_DBG("%d-finger swipe down - INPUT_BTN_SOUTH", num_fingers);
        input_report_key(dev, INPUT_BTN_SOUTH, 1, true, K_FOREVER);
        input_report_key(dev, INPUT_BTN_SOUTH, 0, true, K_FOREVER);
    }
}

/**
 * @brief Checks if the given absolute coordinates are near the trackpad edge
 */
static inline bool tps43_is_near_edge(const struct tps43_config *config, uint16_t x, uint16_t y) {
    int16_t max_x = (config->x_resolution > 0) ? config->x_resolution : 1024;
    int16_t max_y = (config->y_resolution > 0) ? config->y_resolution : 1024;
    int16_t buf = config->drag_continuation_edge_buffer;

    if (config->switch_xy) {
        int16_t tmp = max_x;
        max_x = max_y;
        max_y = tmp;
    }

    return (x <= buf) || (x >= (max_x - buf)) ||
           (y <= buf) || (y >= (max_y - buf));
}

/* Edge drag-continuation auto-slide: motion-injection cadence. */
#define TPS43_SLIDE_TICK_MS 20

/* Integer sqrt for slide direction normalization (this Zephyr has no
 * sys_sqrt32). Integer Newton, 4 iterations, floor-corrected. */
static inline uint32_t tps43_isqrt32(uint32_t v) {
    if (v == 0) {
        return 0;
    }
    uint32_t x = 1U << ((31U - __builtin_clz(v) + 1) / 2);
    for (int i = 0; i < 4; i++) {
        x = (x + v / x) / 2;
    }
    while ((uint64_t)x * x > v) {
        x--;
    }
    return x;
}

/**
 * @brief Delayable work handler for the edge drag-continuation auto-slide
 *
 * One work serves both phases: the armed dwell timer (slide_pending)
 * promotes to an active slide, then the self-rescheduling tick emits
 * constant-speed relative motion away from the pad center along the
 * vector toward the resting finger position. Mutex pattern mirrors
 * tps43_drag_continuation_timeout.
 */
static void tps43_slide_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data =
        CONTAINER_OF(dwork, struct tps43_drv_data, slide_work);
    const struct device *dev = drv_data->dev;
    const struct tps43_config *config = dev->config;

    if (k_mutex_lock(&drv_data->lock, K_MSEC(100)) != 0) {
        /* Same 100 ms retry as the continuation timeout: no busy spin
         * under sustained mutex contention. */
        k_work_reschedule(&drv_data->slide_work, K_MSEC(100));
        return;
    }

    if (drv_data->slide_pending) {
        /* Dwell elapsed. */
        if (!drv_data->drag_active) {
            tps43_slide_stop(drv_data);
            k_mutex_unlock(&drv_data->lock);
            return;
        }
        drv_data->slide_pending = false;
        drv_data->slide_active = true;
        /* Fall through to emit the first tick. */
    }

    if (!drv_data->slide_active) {
        k_mutex_unlock(&drv_data->lock);
        return;
    }

    /* Stop when the drag is gone, a lift armed continuation, the finger
     * count changed, or the buffer was left. */
    if (!drv_data->drag_active || drv_data->drag_continuation_active ||
        drv_data->prev_num_fingers != 1 ||
        !tps43_is_near_edge(config, drv_data->last_abs_x, drv_data->last_abs_y)) {
        tps43_slide_stop(drv_data);
        k_mutex_unlock(&drv_data->lock);
        return;
    }

    /* Direction = pad center -> resting ABS position (integer-normalized).
     * Re-derived every tick, so steering within the buffer is immediate. */
    int16_t max_x = (config->x_resolution > 0) ? config->x_resolution : 1024;
    int16_t max_y = (config->y_resolution > 0) ? config->y_resolution : 1024;
    if (config->switch_xy) {
        int16_t tmp = max_x;
        max_x = max_y;
        max_y = tmp;
    }
    int32_t dx = (int32_t)drv_data->last_abs_x - max_x / 2;
    int32_t dy = (int32_t)drv_data->last_abs_y - max_y / 2;
    uint32_t len = tps43_isqrt32((uint32_t)(dx * dx + dy * dy));
    if (len == 0) {
        /* Defensive: dead-center inside the buffer. Retry next tick. */
        k_work_reschedule(&drv_data->slide_work, K_MSEC(TPS43_SLIDE_TICK_MS));
        k_mutex_unlock(&drv_data->lock);
        return;
    }
    int32_t spd = (int32_t)config->drag_continuation_slide_speed_pps *
                  TPS43_SLIDE_TICK_MS / 1000;
    int16_t sx = (int16_t)((dx * spd) / (int32_t)len);
    int16_t sy = (int16_t)((dy * spd) / (int32_t)len);
    if (sx != 0) {
        input_report_rel(dev, INPUT_REL_X, sx, sy == 0, K_FOREVER);
    }
    if (sy != 0) {
        input_report_rel(dev, INPUT_REL_Y, sy, true, K_FOREVER);
    }

    k_work_reschedule(&drv_data->slide_work, K_MSEC(TPS43_SLIDE_TICK_MS));
    k_mutex_unlock(&drv_data->lock);
}

/**
 * @brief Delayable work handler for drag continuation timeout expiration
 */
static void tps43_drag_continuation_timeout(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data = CONTAINER_OF(dwork, struct tps43_drv_data, drag_continuation_work);
    const struct device *dev = drv_data->dev;

    if (k_mutex_lock(&drv_data->lock, K_MSEC(100)) != 0) {
        /* 100 ms retry avoids a 10 ms spin under sustained mutex contention. */
        k_work_reschedule(&drv_data->drag_continuation_work, K_MSEC(100));
        return;
    }

    if (drv_data->drag_continuation_active) {
        LOG_DBG("Drag continuation timeout expired -> Releasing INPUT_BTN_0");
        drv_data->drag_continuation_active = false;
        drv_data->drag_active = false;
        drv_data->drag_button_pressed = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
        /* Timeout expiry leaves the pad idle: no later report would refresh a
         * button state lost on the wire, so arm the release retry. */
        k_work_reschedule(&drv_data->touch_release_work, K_MSEC(TPS43_TOUCH_RELEASE_RETRY_MS));
    }

    k_mutex_unlock(&drv_data->lock);
}

static void tps43_reconfigure_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data =
        CONTAINER_OF(dwork, struct tps43_drv_data, reconfigure_work);
    const struct device *dev = drv_data->dev;

    if (k_mutex_lock(&drv_data->lock, K_FOREVER) != 0) {
        return;
    }
    if (!drv_data->suspended) {
        tps43_configure_device(dev);
        tps43_end_communication_window(dev);
    } else {
        /* Chip reset while suspended: a bare resume leaves it in factory
         * defaults (no SETUP_COMPLETE -> no reports). Latch; consumed by the
         * next successful resume. */
        drv_data->needs_reconfigure = true;
    }
    k_mutex_unlock(&drv_data->lock);
}

static void tps43_reset_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data =
        CONTAINER_OF(dwork, struct tps43_drv_data, reset_work);
    const struct device *dev = drv_data->dev;

    if (k_mutex_lock(&drv_data->lock, K_FOREVER) != 0) {
        return;
    }
    LOG_INF("Soft reset requested - re-initializing trackpad");
    int ret = tps43_full_reinit(dev);
    if (ret != 0) {
        LOG_ERR("Soft reset failed: %d", ret);
    }
    tps43_end_communication_window(dev);
    k_mutex_unlock(&drv_data->lock);
}

/* Gesture angle props encode as 64*tan(deg) into an 8-bit register. Table at
 * 5-degree granularity (truncated, matching the old float cast exactly) avoids
 * pulling libm into the image; entries above 75 deg would overflow u8 anyway. */
static uint8_t tps43_tan64(int16_t degrees) {
    static const uint8_t lut[] = {0, 5, 11, 17, 23, 29, 36, 44, 53, 64,
                                  76, 91, 110, 137, 175, 238};
    if (degrees < 0) {
        return 0;
    }
    size_t idx = ((size_t)degrees) / 5;
    if (idx >= ARRAY_SIZE(lut)) {
        return UINT8_MAX;
    }
    return lut[idx];
}

/**
 * @brief Main work handler for processing trackpad events
 *
 * Executed when receiving an interrupt from the trackpad (RDY pin).
 * Reads and processes gesture events, cursor movement and scrolling,
 * converting them into input events for the ZMK system.
 * Also manages trackpad wake-up from suspend mode when activity is detected.
 *
 * Protected by mutex to prevent interruption by other I2C operations,
 * which ensures smooth cursor movement without interruptions.
 *
 * @param work Pointer to work structure
 */
static void tps43_work_handler(struct k_work *work) {
    struct tps43_drv_data *drv_data = CONTAINER_OF(work, struct tps43_drv_data, work);
    const struct device *dev = drv_data->dev;
    const struct tps43_config *config = dev->config;
    bool is_scroll_active = false;
    bool is_zoom_active = false;
    bool is_drag_active = drv_data->drag_active;
    int ret;

    // If device is not initialized, ignore interrupt (RDY should be disabled)
    if (!drv_data->initialized) {
        LOG_WRN("RDY interrupt while uninitialized - ignoring");
        return;
    }

    if (drv_data->suspended) {
        LOG_INF("RDY interrupt while suspended - resuming device");
        /* Full wake dance (NACK read, clear SUSPEND, re-arm RDY); a bare flag
         * flip leaves the chip asleep so the first burst read NACKs. */
        if (tps43_set_suspend_internal(dev, false, false) != 0) {
            /* Retry the wake dance now instead of betting the chip re-asserts
             * RDY for the discarded event; a repeat on persistent failure is
             * bounded by the NACK/teardown path below. */
            LOG_WRN("Resume failed, re-queueing handler");
            k_work_submit(&drv_data->work);
        }
    }

    // Acquire mutex to protect all I2C and state operations from interruption
    // This prevents conflicts during simultaneous trackpad access
    k_mutex_lock(&drv_data->lock, K_FOREVER);

    /*
     * Read the event block in a single I2C transaction: up through ABS_Y
     * (13 bytes) when drag continuation is enabled, otherwise 9 bytes up
     * through REL_Y.
     * Byte offsets into the buffer:
     *   [0] GESTURE_EVENTS_0   (0x000D)
     *   [1] GESTURE_EVENTS_1   (0x000E)
     *   [2] SYSTEM_INFO_0      (0x000F)
     *   [3] SYSTEM_INFO_1      (0x0010)
     *   [4] NUM_FINGERS        (0x0011)
     *   [5..6] REL_X           (0x0012, big-endian)
     *   [7..8] REL_Y           (0x0014, big-endian)
     *   [9..10] ABS_X          (0x0016, big-endian, when drag continuation is enabled)
     *   [11..12] ABS_Y         (0x0018, big-endian, when drag continuation is enabled)
     */
    size_t read_len = config->drag_continuation ?
                      ((TPS43_REG_ABS_Y + 2) - TPS43_REG_GESTURE_EVENTS_0) :
                      ((TPS43_REG_REL_Y + 2) - TPS43_REG_GESTURE_EVENTS_0);

    uint8_t touch_data[13];
    ret = read_sequence_registers(dev, TPS43_REG_GESTURE_EVENTS_0, touch_data, read_len);
    /* Immediately free the trackpad so it begins the next capacitive sample cycle */
    tps43_end_communication_window(dev);

    if (ret < 0) {
        if (drv_data->i2c_err_count++ == 0) {
            LOG_WRN("Touch data read error: %d", ret);
        } else {
            LOG_ERR("Touch data read error (x%u): %d", drv_data->i2c_err_count, ret);
        }
        tps43_teardown_session(drv_data);
        is_drag_active = false;
        goto done;
    }
    drv_data->i2c_err_count = 0;

    /* Runtime reset detection and recovery */
    if (unlikely(touch_data[2] & TPS43_SHOW_RESET)) {
        /* AUTO_ATI is a persistent enable: always re-assert it, clearing it
         * disables tuning until the next full init.
         */
        LOG_WRN("Runtime reset detected on trackpad (fingers=%u), reconfiguring...", touch_data[4]);
        tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET | TPS43_AUTO_ATI);
        tps43_end_communication_window(dev);
        /* Deferred so the AUTO_ATI settle does not hold the mutex across a sleep. */
        k_work_reschedule(&drv_data->reconfigure_work,
                          K_MSEC(touch_data[4] == 0 ? 50 : 0));
        tps43_teardown_session(drv_data);
        is_drag_active = false;
        goto done;
    }

    const uint8_t gestures_events[2] = {touch_data[0], touch_data[1]};
    uint8_t num_fingers = touch_data[4];
    int16_t rel_x = (int16_t)((touch_data[5] << 8) | touch_data[6]);
    int16_t rel_y = (int16_t)((touch_data[7] << 8) | touch_data[8]);


    if (config->drag_continuation && num_fingers >= 1) {
        drv_data->last_abs_x = (uint16_t)((touch_data[9] << 8) | touch_data[10]);
        drv_data->last_abs_y = (uint16_t)((touch_data[11] << 8) | touch_data[12]);
    }

    /* Edge drag-continuation auto-slide gate. Runs before the
     * continuation-resume block so slide bookkeeping stays ahead of the
     * lift handling below. Stillness is measured on the ABS delta between
     * consecutive reports, not REL: REL carries chip gesture deltas
     * during an edge drag, the ABS delta is the literal "finger didn't
     * move". */
    if (drv_data->slide_enabled) {
        int32_t sdx = (int32_t)drv_data->last_abs_x - (int32_t)drv_data->slide_prev_abs_x;
        int32_t sdy = (int32_t)drv_data->last_abs_y - (int32_t)drv_data->slide_prev_abs_y;
        int32_t move = (sdx < 0 ? -sdx : sdx) + (sdy < 0 ? -sdy : sdy);
        drv_data->slide_prev_abs_x = drv_data->last_abs_x;
        drv_data->slide_prev_abs_y = drv_data->last_abs_y;
        bool slide_keep = is_drag_active && num_fingers == 1 &&
                          move <= config->drag_continuation_slide_still_px &&
                          tps43_is_near_edge(config, drv_data->last_abs_x,
                                             drv_data->last_abs_y);
        if (slide_keep) {
            if (!drv_data->slide_active && !drv_data->slide_pending) {
                drv_data->slide_pending = true;
                k_work_reschedule(&drv_data->slide_work,
                                  K_MSEC(config->drag_continuation_slide_dwell_ms));
            }
            /* Pending or active: leave the timer alone -- the dwell must
             * elapse uninterrupted, and while active the self-rescheduling
             * tick handles steering. */
        } else if (drv_data->slide_pending || drv_data->slide_active) {
            tps43_slide_stop(drv_data);
        }
    }

    /* In-loop self-healing and continuation resumption */
    if (drv_data->drag_continuation_active) {
        k_work_cancel_delayable(&drv_data->drag_continuation_work);

        if (num_fingers == 1) {
            LOG_DBG("Drag continuation resumed from reposition");
            drv_data->drag_continuation_active = false;
            is_drag_active = true;
        } else {
            LOG_DBG("Continuation aborted by finger count (%d) -> Releasing button", num_fingers);
            tps43_slide_stop(drv_data);
            drv_data->drag_continuation_active = false;
            drv_data->drag_active = false;
            is_drag_active = false;
            if (drv_data->drag_button_pressed) {
                drv_data->drag_button_pressed = false;
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
            }
        }
    } else if (drv_data->drag_button_pressed && !is_drag_active) {
        LOG_WRN("Orphaned drag button state detected -> Force clearing INPUT_BTN_0");
        drv_data->drag_button_pressed = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    }

    bool is_touching = (num_fingers > 0);
    if (is_touching != drv_data->touching) {
        LOG_DBG("Touch state changed: %s", is_touching ? "down" : "up");
        if (is_touching) {
            drv_data->touching = true;
            input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
        } else {
            tps43_release_touch(dev);
        }
    }

    /* Track peak finger count for the current touch session */
    if (num_fingers > drv_data->touch_session_fingers) {
        drv_data->touch_session_fingers = num_fingers;
    }

    if (gestures_events[0] != 0 || gestures_events[1] != 0) {

        LOG_DBG("Gestures: Single=0x%02X, Multi=0x%02X", gestures_events[0], gestures_events[1]);

        if (gestures_events[0] & TPS43_SINGLE_TAP) {
            LOG_DBG("Single tap → LEFT BUTTON");
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
            input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
        }
        if (gestures_events[0] & (TPS43_SWIPE_UP | TPS43_SWIPE_DOWN | TPS43_SWIPE_LEFT | TPS43_SWIPE_RIGHT)) {
            LOG_DBG("Single finger swipe");
            tps43_handle_swipe(dev, num_fingers, rel_x, rel_y);
        }
        if (gestures_events[1] & TPS43_TWO_FINGER_TAP) {
            if (config->three_finger_tap && drv_data->touch_session_fingers >= 3) {
                LOG_DBG("Three finger tap -> MIDDLE BUTTON (INPUT_BTN_2)");
                input_report_key(dev, INPUT_BTN_2, 1, true, K_FOREVER);
                input_report_key(dev, INPUT_BTN_2, 0, true, K_FOREVER);
            } else if (config->two_finger_tap) {
                LOG_DBG("Two finger tap -> RIGHT BUTTON (INPUT_BTN_1)");
                input_report_key(dev, INPUT_BTN_1, 1, true, K_FOREVER);
                input_report_key(dev, INPUT_BTN_1, 0, true, K_FOREVER);
            }
        }
        if ((gestures_events[0] & TPS43_PRESS_AND_HOLD) && !is_drag_active) {
            LOG_DBG("Press and hold detected - DRAG (HOLD LEFT BUTTON)");
            // set internal drag flag and press left mouse button
            is_drag_active = true;
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
        }
        if (gestures_events[1] & TPS43_SCROLL) {
            // set scroll flag for processing in tp_movement block
            is_scroll_active = true;
        }
        if (gestures_events[1] & TPS43_ZOOM) {
            // set zoom flag for processing in tp_movement block
            is_zoom_active = true;
        }
    }

    /* Release drag lock when contact is lost */
    if (drv_data->prev_num_fingers > 0 && num_fingers == 0) {
        if (config->drag_continuation && is_drag_active &&
            tps43_is_near_edge(config, drv_data->last_abs_x, drv_data->last_abs_y)) {

            LOG_DBG("Edge lift detected -> Arming drag continuation (%d ms)",
                    config->drag_continuation_timeout_ms);
            drv_data->drag_continuation_active = true;
            k_work_reschedule(&drv_data->drag_continuation_work,
                              K_MSEC(config->drag_continuation_timeout_ms));
            /* Keep is_drag_active intact and do not emit button release */
        } else {
            if (is_drag_active) {
                LOG_DBG("Drag contact lost -> RELEASE LEFT BUTTON");
                is_drag_active = false;
                drv_data->drag_button_pressed = false;
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
            }
        }
    }

    /* Track drag_button_pressed state when any drag mode is active */
    if (is_drag_active || drv_data->drag_continuation_active) {
        drv_data->drag_button_pressed = true;
    } else {
        drv_data->drag_button_pressed = false;
    }

    if (rel_x != 0 || rel_y != 0) {
        if (num_fingers == 3) {
            LOG_DBG("Three-finger movement - checking for swipe");
            tps43_handle_swipe(dev, num_fingers, rel_x, rel_y);
        } else if (is_scroll_active) {
            // Scroll processing: keep only dominant axis
            if ((rel_x < 0 ? -rel_x : rel_x) > (rel_y < 0 ? -rel_y : rel_y)) {
                // Horizontal scroll
                if (config->invert_scroll_x) {
                    rel_x = -rel_x;
                }
                int16_t wheel = (rel_x * config->scroll_sensitivity) / 100;
                if (wheel != 0) {
                    LOG_DBG("Scrolling %d horizontally", wheel);
                    input_report_rel(dev, INPUT_REL_HWHEEL, wheel, true, K_FOREVER);
                }
            } else {
                // Vertical scroll
                if (config->invert_scroll_y) {
                    rel_y = -rel_y;
                }
                int16_t wheel = (rel_y * config->scroll_sensitivity) / 100;
                if (wheel != 0) {
                    LOG_DBG("Scrolling %d vertically", wheel);
                    input_report_rel(dev, INPUT_REL_WHEEL, wheel, true, K_FOREVER);
                }
            }
            is_scroll_active = false;
        } else if (is_zoom_active) {
            // Zoom processing: the zoom amount comes in via rel_x
            int16_t zoom_delta = (rel_x * config->zoom_sensitivity) / 100;
            if (zoom_delta != 0) {
                LOG_DBG("Zooming %d, rel_x=%d", zoom_delta, rel_x);
                input_report_rel(dev, INPUT_REL_MISC, zoom_delta, true, K_FOREVER);
            }
            is_zoom_active = false;
        } else {
            // Normal cursor movement
            if (rel_x != 0) {
                int32_t scaled_x = ((int32_t)rel_x * config->sensitivity) / 100;
                rel_x = (int16_t)CLAMP(scaled_x, INT16_MIN, INT16_MAX);
            }
            if (rel_y != 0) {
                int32_t scaled_y = ((int32_t)rel_y * config->sensitivity) / 100;
                rel_y = (int16_t)CLAMP(scaled_y, INT16_MIN, INT16_MAX);
            }
            LOG_DBG("Sending movement: dx=%d, dy=%d", rel_x, rel_y);

            if (rel_x != 0) {
                input_report_rel(dev, INPUT_REL_X, rel_x, (rel_y == 0), K_FOREVER);
            }
            if (rel_y != 0) {
                input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
            }
        }
    }

    if (num_fingers == 0) {
        drv_data->touch_session_fingers = 0;
    }

    drv_data->prev_num_fingers = num_fingers;

done:
    // Save for next call
    drv_data->drag_active = is_drag_active;

    // Release mutex after completing event handling
    k_mutex_unlock(&drv_data->lock);
}

/**
 * @brief Resets driver internal state values
 *
 * Initializes all driver state flags to initial values.
 * Used during initialization and device reset.
 *
 * @param dev Pointer to trackpad device
 * @return 0 on success
 */
static int tps43_reset_values(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;

    k_work_cancel_delayable(&drv_data->drag_continuation_work);
    drv_data->drag_continuation_active = false;
    k_work_cancel_delayable(&drv_data->slide_work);
    drv_data->slide_pending = false;
    drv_data->slide_active = false;

    if (drv_data->touching) {
        drv_data->touching = false;
        input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    }

    if (drv_data->drag_button_pressed || drv_data->drag_active) {
        drv_data->drag_button_pressed = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    }

    drv_data->initialized = false;
    drv_data->drag_active = false;
    drv_data->touching = false;
    drv_data->touch_session_fingers = 0;
    drv_data->prev_num_fingers = 0;
    drv_data->last_abs_x = 0;
    drv_data->last_abs_y = 0;

    LOG_INF("Values reset");
    return 0;
}

/**
 * @brief Configures trackpad system registers for operation
 *
 * Sets up trackpad registers to track touch events, gestures and movement.
 * Enables necessary gestures (single tap, press and hold, scroll, two finger tap),
 * configures axis inversion and sets setup complete flag.
 *
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int tps43_configure_device(const struct device *dev) {

    const struct tps43_config *config = dev->config;
    int ret;

    // write to TPS43_REG_SYSTEM_CONFIG_1 events to track
    uint8_t events_to_track = TPS43_TP_EVENT | TPS43_EVENT_MODE;

    // Gestures (single_tap, press_and_hold, scroll, two_finger_tap)
    if (config->single_tap || config->press_and_hold ||
        config->scroll || config->two_finger_tap) {
        events_to_track |= TPS43_GESTURE_EVENT;
    }

    // Touch events for absolute coordinates
    events_to_track |= TPS43_TOUCH_EVENT;

    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, events_to_track);
    if (ret != 0) {
        LOG_WRN("Events to track write error: %d", ret);
        return ret;
    }
    LOG_INF("Events configured: 0x%02X", events_to_track);

    // axis configuration
    uint8_t xy_config = 0;
    xy_config |= config->invert_x ? TPS43_FLIP_X : 0;
    xy_config |= config->invert_y ? TPS43_FLIP_Y : 0;
    xy_config |= config->switch_xy ? TPS43_SWITCH_XY_AXIS : 0;
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_CONFIG_0, xy_config);
    if (ret != 0) {
        LOG_WRN("XY configuration write error: %d", ret);
        return ret;
    }

    // enable single gestures at hardware level
    if (config->single_tap || config->press_and_hold || config->swipes || config->three_finger_swipe) {
        uint8_t single_gestures = 0;
        single_gestures |= config->single_tap ? TPS43_SINGLE_TAP : 0;
        single_gestures |= config->press_and_hold ? TPS43_PRESS_AND_HOLD : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_UP : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_DOWN : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_LEFT : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_RIGHT : 0;

        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SINGLE_FINGER_GESTURES, single_gestures);
        if (ret != 0) {
            LOG_WRN("Single gestures configuration error: %d", ret);
            return ret;
        }
        LOG_INF("Single gestures enabled: 0x%02X", single_gestures);
    }

    // enable multi-gestures
    if (config->two_finger_tap || config->scroll || config->zoom) {
        uint8_t multi_gestures = 0;
        multi_gestures |= config->two_finger_tap ? TPS43_TWO_FINGER_TAP : 0;
        multi_gestures |= config->scroll ? TPS43_SCROLL : 0;
        multi_gestures |= config->zoom ? TPS43_ZOOM : 0;

        ret = tps43_i2c_write_reg8(dev, TPS43_REG_MULTI_FINGER_GESTURES, multi_gestures);
        if (ret != 0) {
            LOG_WRN("Multi-gesture configuration error: %d", ret);
            return ret;
        }
        LOG_INF("Multi-gestures enabled: 0x%02X", multi_gestures);
    }

    // filter configuration
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_FILTER_SETTINGS, config->filter_settings);
    if (ret != 0) {
        LOG_WRN("Filter settings write error: %d", ret);
        return ret;
    }
    LOG_INF("Filter settings set: 0x%02X", config->filter_settings);

    // dynamic filter configuration (only if set in DT)
    if (config->filter_dynamic_bottom != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM,
                                   (uint8_t)config->filter_dynamic_bottom);
        if (ret != 0) {
            LOG_WRN("Dynamic filter bottom write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter bottom set: 0x%02X", (uint8_t)config->filter_dynamic_bottom);
    }

    if (config->filter_dynamic_lower != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_LOWER,
                                   (uint8_t)config->filter_dynamic_lower);
        if (ret != 0) {
            LOG_WRN("Dynamic filter lower write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter lower set: 0x%02X", (uint8_t)config->filter_dynamic_lower);
    }

    if (config->filter_dynamic_upper != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_XY_DYNAMIC_FILTER_UPPER,
                                    (uint16_t)config->filter_dynamic_upper);
        if (ret != 0) {
            LOG_WRN("Dynamic filter upper write error: %d", ret);
            return ret;
        }
        LOG_INF("Dynamic filter upper set: 0x%04X", (uint16_t)config->filter_dynamic_upper);
    }

    // resolution configuration (only if set in DT)
    if (config->x_resolution > 0) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_X_RESOLUTION,
                                    (uint16_t)config->x_resolution);
        if (ret != 0) {
            LOG_WRN("X resolution write error: %d", ret);
            return ret;
        }
        LOG_INF("X resolution set: %d", (uint16_t)config->x_resolution);
    }

    if (config->y_resolution > 0) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_Y_RESOLUTION,
                                    (uint16_t)config->y_resolution);
        if (ret != 0) {
            LOG_WRN("Y resolution write error: %d", ret);
            return ret;
        }
        LOG_INF("Y resolution set: %d", (uint16_t)config->y_resolution);
    }

    // swipe configuration (only if set in DT)
    if (config->swipe_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_INITIAL_DISTANCE,
                                    (uint16_t)config->swipe_initial_distance);
        if (ret != 0) {
            LOG_WRN("Swipe distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe initial distance set: %u px", (uint16_t)config->swipe_initial_distance);
    }

    if (config->swipe_initial_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_INITIAL_TIME,
                                    (uint16_t)config->swipe_initial_time);
        if (ret != 0) {
            LOG_WRN("Swipe time write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe initial time set: %u ms", (uint16_t)config->swipe_initial_time);
    }

    if (config->swipe_angle != -1) {
        uint8_t reg_val = tps43_tan64(config->swipe_angle);
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SWIPE_ANGLE, reg_val);
        if (ret != 0) {
            LOG_WRN("Swipe angle write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe angle set: %d deg (reg: 0x%02X)", config->swipe_angle, reg_val);
    }

    if (config->swipe_consecutive_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE,
                                    (uint16_t)config->swipe_consecutive_distance);
        if (ret != 0) {
            LOG_WRN("Swipe consecutive distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe consecutive distance set: %u px", (uint16_t)config->swipe_consecutive_distance);
    }

    if (config->swipe_consecutive_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_TIME,
                                    (uint16_t)config->swipe_consecutive_time);
        if (ret != 0) {
            LOG_WRN("Swipe consecutive time write error: %d", ret);
            return ret;
        }
        LOG_INF("Swipe consecutive time set: %u ms", (uint16_t)config->swipe_consecutive_time);
    }

    // scroll configuration (only if set in DT)
    if (config->scroll_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_SCROLL_INITIAL_DISTANCE,
                                    (uint16_t)config->scroll_initial_distance);
        if (ret != 0) {
            LOG_WRN("Scroll initial distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Scroll initial distance set: %u px", (uint16_t)config->scroll_initial_distance);
    }

    if (config->scroll_angle != -1) {
        uint8_t reg_val = tps43_tan64(config->scroll_angle);
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SCROLL_ANGLE, reg_val);
        if (ret != 0) {
            LOG_WRN("Scroll angle write error: %d", ret);
            return ret;
        }
        LOG_INF("Scroll angle set: %d deg (reg: 0x%02X)", config->scroll_angle, reg_val);
    }

    // zoom configuration (only if set in DT)
    if (config->zoom_initial_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ZOOM_INITIAL_DISTANCE,
                                    (uint16_t)config->zoom_initial_distance);
        if (ret != 0) {
            LOG_WRN("Zoom initial distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Zoom initial distance set: %u px", (uint16_t)config->zoom_initial_distance);
    }

    if (config->zoom_consecutive_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE,
                                    (uint16_t)config->zoom_consecutive_distance);
        if (ret != 0) {
            LOG_WRN("Zoom consecutive distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Zoom consecutive distance set: %u px", (uint16_t)config->zoom_consecutive_distance);
    }

    // tap/hold configuration (only if set in DT)
    if (config->tap_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_TIME,
                                    (uint16_t)config->tap_time);
        if (ret != 0) {
            LOG_WRN("Tap time write error: %d", ret);
            return ret;
        }
        LOG_INF("Tap time set: %u ms", (uint16_t)config->tap_time);
    }

    if (config->tap_distance != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_DISTANCE,
                                    (uint16_t)config->tap_distance);
        if (ret != 0) {
            LOG_WRN("Tap distance write error: %d", ret);
            return ret;
        }
        LOG_INF("Tap distance set: %u px", (uint16_t)config->tap_distance);
    }

    if (config->hold_time != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_HOLD_TIME,
                                    (uint16_t)config->hold_time);
        if (ret != 0) {
            LOG_WRN("Hold time write error: %d", ret);
            return ret;
        }
        LOG_INF("Hold time set: %u ms", (uint16_t)config->hold_time);
    }

    // ATI configuration (only if set in DT)
    if (config->ati_target != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_ATI_TARGET,
                                    (uint16_t)config->ati_target);
        if (ret != 0) {
            LOG_WRN("ATI target write error: %d", ret);
            return ret;
        }
        LOG_INF("ATI target set: %u", (uint16_t)config->ati_target);
    }

    if (config->ref_drift_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REF_DRIFT_LIMIT,
                                   (uint8_t)config->ref_drift_limit);
        if (ret != 0) {
            LOG_WRN("Ref drift limit write error: %d", ret);
            return ret;
        }
        LOG_INF("Ref drift limit set: %u", (uint8_t)config->ref_drift_limit);
    }

    if (config->reati_lower_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REATI_LOWER_LIMIT,
                                   (uint8_t)config->reati_lower_limit);
        if (ret != 0) {
            LOG_WRN("REATI lower limit write error: %d", ret);
            return ret;
        }
        LOG_INF("REATI lower limit set: %u", (uint8_t)config->reati_lower_limit);
    }

    if (config->reati_upper_limit != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REATI_UPPER_LIMIT,
                                   (uint8_t)config->reati_upper_limit);
        if (ret != 0) {
            LOG_WRN("REATI upper limit write error: %d", ret);
            return ret;
        }
        LOG_INF("REATI upper limit set: %u", (uint8_t)config->reati_upper_limit);
    }

    if (config->max_count_limit != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_MAX_COUNT_LIMIT,
                                    (uint16_t)config->max_count_limit);
        if (ret != 0) {
            LOG_WRN("Max count limit write error: %d", ret);
            return ret;
        }
        LOG_INF("Max count limit set: %u", (uint16_t)config->max_count_limit);
    }

    if (config->ati_retry_time != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_ATI_RETRY_TIME,
                                   (uint8_t)config->ati_retry_time);
        if (ret != 0) {
            LOG_WRN("ATI retry time write error: %d", ret);
            return ret;
        }
        LOG_INF("ATI retry time set: %u s", (uint8_t)config->ati_retry_time);
    }

    // Report rate configuration (only if set in DT)
    if (config->report_rate_active != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_ACTIVE,
                                    (uint16_t)config->report_rate_active);
        if (ret != 0) {
            LOG_WRN("Report rate active write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate active set: %u ms", (uint16_t)config->report_rate_active);
    }

    if (config->report_rate_idle_touch != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_IDLE_TOUCH,
                                    (uint16_t)config->report_rate_idle_touch);
        if (ret != 0) {
            LOG_WRN("Report rate idle touch write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate idle touch set: %u ms", (uint16_t)config->report_rate_idle_touch);
    }

    if (config->report_rate_idle != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_IDLE,
                                    (uint16_t)config->report_rate_idle);
        if (ret != 0) {
            LOG_WRN("Report rate idle write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate idle set: %u ms", (uint16_t)config->report_rate_idle);
    }

    if (config->report_rate_lp1 != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_LP1,
                                    (uint16_t)config->report_rate_lp1);
        if (ret != 0) {
            LOG_WRN("Report rate LP1 write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate LP1 set: %u ms", (uint16_t)config->report_rate_lp1);
    }

    if (config->report_rate_lp2 != -1) {
        ret = tps43_i2c_write_reg16(dev, TPS43_REG_REPORT_RATE_LP2,
                                    (uint16_t)config->report_rate_lp2);
        if (ret != 0) {
            LOG_WRN("Report rate LP2 write error: %d", ret);
            return ret;
        }
        LOG_INF("Report rate LP2 set: %u ms", (uint16_t)config->report_rate_lp2);
    }

    // Timeout configuration (only if set in DT)
    if (config->timeout_active != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_ACTIVE,
                                   (uint8_t)config->timeout_active);
        if (ret != 0) {
            LOG_WRN("Timeout active write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout active set: %u s", (uint8_t)config->timeout_active);
    }

    if (config->timeout_idle_touch != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_IDLE_TOUCH,
                                   (uint8_t)config->timeout_idle_touch);
        if (ret != 0) {
            LOG_WRN("Timeout idle touch write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout idle touch set: %u s", (uint8_t)config->timeout_idle_touch);
    }

    if (config->timeout_idle != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_IDLE,
                                   (uint8_t)config->timeout_idle);
        if (ret != 0) {
            LOG_WRN("Timeout idle write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout idle set: %u s", (uint8_t)config->timeout_idle);
    }

    if (config->timeout_lp1 != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_TIMEOUT_LP1,
                                   (uint8_t)config->timeout_lp1);
        if (ret != 0) {
            LOG_WRN("Timeout LP1 write error: %d", ret);
            return ret;
        }
        LOG_INF("Timeout LP1 set: %u s", (uint8_t)config->timeout_lp1);
    }

    if (config->ref_update_time != -1) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_REF_UPDATE_TIME,
                                   (uint8_t)config->ref_update_time);
        if (ret != 0) {
            LOG_WRN("Ref update time write error: %d", ret);
            return ret;
        }
        LOG_INF("Ref update time set: %u s", (uint8_t)config->ref_update_time);
    }

    // set configuration complete flag
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, TPS43_SETUP_COMPLETE | TPS43_WDT_ENABLE | TPS43_REATI);
    if (ret != 0) {
        LOG_WRN("Setup complete flag write error: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Checks device reset state and performs reconfiguration
 *
 * Waits for device readiness after reset, checks SHOW_RESET flag
 * and sends reset acknowledgment (ACK_RESET) when necessary.
 * Then performs full device configuration.
 *
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int check_reset_and_reconfigure(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    int ret;
    uint8_t sys_info = 0;
    uint8_t wait_count = 0;
    const uint8_t max_wait_count = 50;

    // Wait for device readiness (the first few reads may fail until device is up)
    do {
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info);
        if (ret < 0) {
            k_sleep(K_MSEC(100));
            wait_count++;
            if (wait_count >= max_wait_count) {
                LOG_ERR("Device not responding after %d ms", wait_count * 100);
                return -ETIMEDOUT;
            }
        }
    } while (ret < 0);

    LOG_INF("Device ready after %d ms", wait_count * 100);

    // after reset, set flag to acknowledge that reset was performed.
    // Stay inside the setup window: the config writes below and the closing
    // register dump must land in the same window the ACK opened.
    if (sys_info & TPS43_SHOW_RESET) {
        LOG_INF("SHOW_RESET detected, sending ACK_RESET and enable ATI");
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET | TPS43_AUTO_ATI);
        if (ret != 0) {
            LOG_ERR("ACK_RESET send error: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(10));
    }

    ret = tps43_configure_device(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration error: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Full chip and driver-state re-initialization
 *
 * Shared core of the boot path (tps43_init) and the manual soft reset
 * (reset_work): clears driver state and stuck buttons, verifies the I2C
 * bus, pulses the hardware RST pin, re-acknowledges the chip reset and
 * rewrites the full configuration, then re-arms the RDY interrupt.
 * Blocking: the RST pulse sleeps ~620 ms and the readiness poll can wait
 * up to 5 s; callers on the shared workqueue must hold the driver mutex
 * across the whole call.
 *
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int tps43_full_reinit(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    ret = tps43_reset_values(dev);
    if (ret != 0) {
        LOG_ERR("Values reset error: %d", ret);
        return ret;
    }

    // Check I2C bus
    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_ERR("I2C bus not available");
        return -ENODEV;
    }

    LOG_INF("I2C bus: %s", config->i2c_bus.bus->name);
    LOG_INF("I2C address: 0x%02x", config->i2c_bus.addr);

    // GPIO reset via hardware RST
    if (config->rst_gpio.port) {
        ret = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("RST GPIO configuration error: %d", ret);
            return ret;
        }

        gpio_pin_set_dt(&config->rst_gpio, 0);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&config->rst_gpio, 1);
        k_sleep(K_MSEC(610));

        LOG_INF("Hardware reset completed");
    }

    // check SHOW_RESET and configure
    ret = check_reset_and_reconfigure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration error: %d", ret);
        return ret;
    }

    drv_data->initialized = true;
    drv_data->suspended = false;

    // configure RDY interrupts only AFTER device configuration!
    if (config->rdy_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_WRN("RDY GPIO configuration error: %d", ret);
        } else {
            ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio,
                                                    GPIO_INT_EDGE_TO_ACTIVE);
            if (ret == 0) {
                /* Manual re-init re-enters here: the core rejects
                 * re-adding an already-registered callback, so drop the
                 * previous registration (no-op on the first call) before
                 * re-adding; otherwise the pending-RDY poll below would be
                 * skipped and the pad would look dead after a soft reset.
                 * Init must precede remove: the shared helper asserts a
                 * non-NULL handler, which a fresh boot has not set yet. */
                gpio_init_callback(&drv_data->rdy_cb, tps43_rdy_callback,
                                    BIT(config->rdy_gpio.pin));
                (void)gpio_remove_callback(config->rdy_gpio.port,
                                           &drv_data->rdy_cb);
                ret = gpio_add_callback(config->rdy_gpio.port, &drv_data->rdy_cb);
                if (ret == 0) {
                    LOG_INF("RDY interrupt configured");
                    if (gpio_pin_get_dt(&config->rdy_gpio) > 0) {
                        k_work_submit(&drv_data->work);
                    }
                } else {
                    LOG_WRN("RDY callback add error: %d", ret);
                }
            }
        }
    }

    return 0;
}

/**
 * @brief Public function to put trackpad into suspend/resume
 *
 * @param dev Pointer to trackpad device
 * @param suspend true - enter suspend, false - exit suspend
 * @return 0 on success, negative error code on failure
 */
static int tps43_set_suspend(const struct device *dev, bool suspend) {
    return tps43_set_suspend_internal(dev, suspend, false);
}

static void tps43_dump_registers(const struct device *dev) {
    uint8_t reg8;
    uint16_t reg16;
    int ret = 0;
    int read_ret;

    LOG_INF("Dumping TPS43 registers");

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_INFO_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_INFO_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_INFO_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_INFO_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONTROL_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONTROL_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONTROL_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONTROL_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONFIG_0 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONFIG_0, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SYSTEM_CONFIG_1 (0x%04X): 0x%02X", TPS43_REG_SYSTEM_CONFIG_1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_GLOBAL_ATI_C, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("GLOBAL_ATI_C (0x%04X): 0x%02X", TPS43_REG_GLOBAL_ATI_C, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ATI_TARGET, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ATI_TARGET (0x%04X): 0x%04X", TPS43_REG_ATI_TARGET, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REF_DRIFT_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REF_DRIFT_LIMIT (0x%04X): 0x%02X", TPS43_REG_REF_DRIFT_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REATI_LOWER_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REATI_LOWER_LIMIT (0x%04X): 0x%02X", TPS43_REG_REATI_LOWER_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REATI_UPPER_LIMIT, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REATI_UPPER_LIMIT (0x%04X): 0x%02X", TPS43_REG_REATI_UPPER_LIMIT, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_MAX_COUNT_LIMIT, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("MAX_COUNT_LIMIT (0x%04X): 0x%04X", TPS43_REG_MAX_COUNT_LIMIT, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ATI_RETRY_TIME, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ATI_RETRY_TIME (0x%04X): 0x%02X", TPS43_REG_ATI_RETRY_TIME, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_ACTIVE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_ACTIVE (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_ACTIVE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_IDLE_TOUCH, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_IDLE_TOUCH (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_IDLE_TOUCH, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_IDLE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_IDLE (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_IDLE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_LP1, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_LP1 (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_LP1, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_REPORT_RATE_LP2, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REPORT_RATE_LP2 (0x%04X): 0x%04X", TPS43_REG_REPORT_RATE_LP2, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_ACTIVE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_ACTIVE (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_ACTIVE, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_IDLE_TOUCH, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_IDLE_TOUCH (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_IDLE_TOUCH, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_IDLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_IDLE (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_IDLE, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_TIMEOUT_LP1, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TIMEOUT_LP1 (0x%04X): 0x%02X", TPS43_REG_TIMEOUT_LP1, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_REF_UPDATE_TIME, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("REF_UPDATE_TIME (0x%04X): 0x%02X", TPS43_REG_REF_UPDATE_TIME, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_STATIC_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_STATIC_BETA (0x%04X): 0x%02X", TPS43_REG_XY_STATIC_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP_COUNT_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP_COUNT_BETA (0x%04X): 0x%02X", TPS43_REG_ALP_COUNT_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP1_LTA_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP1_LTA_BETA (0x%04X): 0x%02X", TPS43_REG_ALP1_LTA_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_ALP2_LTA_BETA, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ALP2_LTA_BETA (0x%04X): 0x%02X", TPS43_REG_ALP2_LTA_BETA, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_BOTTOM (0x%04X): 0x%02X", TPS43_REG_XY_DYNAMIC_FILTER_BOTTOM, reg8);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_XY_DYNAMIC_FILTER_LOWER, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_LOWER (0x%04X): 0x%02X", TPS43_REG_XY_DYNAMIC_FILTER_LOWER, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_XY_DYNAMIC_FILTER_UPPER, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("XY_DYNAMIC_FILTER_UPPER (0x%04X): 0x%04X", TPS43_REG_XY_DYNAMIC_FILTER_UPPER, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_X_RESOLUTION, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("X_RESOLUTION (0x%04X): 0x%04X", TPS43_REG_X_RESOLUTION, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_Y_RESOLUTION, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("Y_RESOLUTION (0x%04X): 0x%04X", TPS43_REG_Y_RESOLUTION, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_TAP_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TAP_TIME (0x%04X): %u ms", TPS43_REG_TAP_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_TAP_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("TAP_DISTANCE (0x%04X): %u px", TPS43_REG_TAP_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_HOLD_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("HOLD_TIME (0x%04X): %u ms", TPS43_REG_HOLD_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_INITIAL_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_INITIAL_TIME (0x%04X): %u ms", TPS43_REG_SWIPE_INITIAL_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_SWIPE_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_TIME, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_CONSECUTIVE_TIME (0x%04X): %u ms", TPS43_REG_SWIPE_CONSECUTIVE_TIME, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_CONSECUTIVE_DISTANCE (0x%04X): %u px", TPS43_REG_SWIPE_CONSECUTIVE_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SWIPE_ANGLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SWIPE_ANGLE (0x%04X): 0x%02X", TPS43_REG_SWIPE_ANGLE, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_SCROLL_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SCROLL_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_SCROLL_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg8(dev, TPS43_REG_SCROLL_ANGLE, &reg8);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("SCROLL_ANGLE (0x%04X): 0x%02X", TPS43_REG_SCROLL_ANGLE, reg8);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ZOOM_INITIAL_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ZOOM_INITIAL_DISTANCE (0x%04X): %u px", TPS43_REG_ZOOM_INITIAL_DISTANCE, reg16);
    }

    read_ret = tps43_i2c_read_reg16(dev, TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE, &reg16);
    ret |= read_ret;
    if (read_ret == 0) {
        LOG_INF("ZOOM_CONSECUTIVE_DISTANCE (0x%04X): %u px", TPS43_REG_ZOOM_CONSECUTIVE_DISTANCE, reg16);
    }

    if (ret != 0) {
        LOG_WRN("error: some reads failed...");
    }

    tps43_end_communication_window(dev);
}

/**
 * @brief Initializes TPS43 trackpad driver
 *
 * Performs full driver initialization: checks I2C bus availability,
 * performs hardware reset via GPIO RST (if connected), waits for device
 * readiness, configures trackpad registers and sets up GPIO RDY interrupts.
 * Also initializes power management system when necessary.
 *
 * @param dev Pointer to trackpad device
 * @return 0 on success, negative error code on failure
 */
static int tps43_init(const struct device *dev) {

    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    drv_data->dev = dev;

    LOG_INF("=== Azoteq tps43 driver for device %s ===", dev->name);

    // Initialize synchronization and work queues early
    k_mutex_init(&drv_data->lock);
    k_work_init(&drv_data->work, tps43_work_handler);
    k_work_init_delayable(&drv_data->drag_continuation_work, tps43_drag_continuation_timeout);
    k_work_init_delayable(&drv_data->touch_release_work, tps43_touch_release_retry);
    k_work_init_delayable(&drv_data->reconfigure_work, tps43_reconfigure_work_handler);
    k_work_init_delayable(&drv_data->slide_work, tps43_slide_work_handler);
    k_work_init_delayable(&drv_data->reset_work, tps43_reset_work_handler);

    drv_data->slide_enabled =
        config->drag_continuation_slide && config->drag_continuation;
    if (config->drag_continuation_slide && !config->drag_continuation) {
        LOG_WRN("drag-continuation-slide requires drag-continuation - disabled");
    }

    // Hardware reset, chip re-configuration and RDY IRQ re-arm
    ret = tps43_full_reinit(dev);
    if (ret != 0) {
        return ret;
    }

    tps43_dump_registers(dev);

    LOG_INF("TPS43 driver successfully initialized");
    return 0;
}


#define TPS43_INIT(inst)                                                                             \
    static struct tps43_drv_data tps43_##inst##_drvdata = {                                          \
        .initialized = false,                                                                        \
        .drag_active = false,                                                                        \
        .drag_continuation_active = false,                                                           \
        .drag_button_pressed = false,                                                                \
        .last_abs_x = 0,                                                                             \
        .last_abs_y = 0,                                                                             \
        .slide_pending = false,                                                                        \
        .slide_active = false,                                                                         \
        .slide_prev_abs_x = 0,                                                                         \
        .slide_prev_abs_y = 0,                                                                         \
        .suspended = false,                                                                          \
        .touching = false,                                                                           \
        .touch_session_fingers = 0,                                                                  \
        .prev_num_fingers = 0,                                                                       \
    };                                                                                               \
                                                                                                     \
    static const struct tps43_config tps43_##inst##_config = {                                       \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                                       \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {0}),                                  \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),                                  \
        .single_tap = DT_INST_PROP(inst, single_tap),                                                \
        .drag_continuation = DT_INST_PROP_OR(inst, drag_continuation, false),                         \
        .drag_continuation_timeout_ms = DT_INST_PROP_OR(inst, drag_continuation_timeout_ms, 600),    \
        .drag_continuation_edge_buffer = DT_INST_PROP_OR(inst, drag_continuation_edge_buffer, 80),   \
        .drag_continuation_slide = DT_INST_PROP_OR(inst, drag_continuation_slide, false),              \
        .drag_continuation_slide_dwell_ms = DT_INST_PROP_OR(inst, drag_continuation_slide_dwell_ms, 250), \
        .drag_continuation_slide_still_px = DT_INST_PROP_OR(inst, drag_continuation_slide_still_px, 12),  \
        .drag_continuation_slide_speed_pps = DT_INST_PROP_OR(inst, drag_continuation_slide_speed, 400), \
        .press_and_hold = DT_INST_PROP(inst, press_and_hold),                                        \
        .two_finger_tap = DT_INST_PROP(inst, two_finger_tap),                                        \
        .three_finger_tap = DT_INST_PROP(inst, three_finger_tap),                                    \
        .scroll = DT_INST_PROP(inst, scroll),                                                        \
        .zoom = DT_INST_PROP(inst, zoom),                                                            \
        .swipes = DT_INST_PROP(inst, swipes),                                                        \
        .three_finger_swipe = DT_INST_PROP(inst, three_finger_swipe),                                \
        .three_finger_swipe_throttle_ms = DT_INST_PROP_OR(inst, three_finger_swipe_throttle_ms, 300),\
        .invert_x = DT_INST_PROP(inst, invert_x),                                                    \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                    \
        .switch_xy = DT_INST_PROP(inst, switch_xy),                                                  \
        .invert_scroll_x = DT_INST_PROP(inst, invert_scroll_x),                                      \
        .invert_scroll_y = DT_INST_PROP(inst, invert_scroll_y),                                      \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 100),                                      \
        .scroll_sensitivity = DT_INST_PROP_OR(inst, scroll_sensitivity, 100),                        \
        .zoom_sensitivity = DT_INST_PROP_OR(inst, zoom_sensitivity, 100),                            \
        .enable_power_management = DT_INST_PROP_OR(inst, enable_power_management, true),             \
        .idle_sleep = DT_INST_PROP_OR(inst, idle_sleep, false),                                      \
        .filter_settings = DT_INST_PROP_OR(inst, filter_settings, 0x0F),                             \
        .filter_dynamic_bottom = DT_INST_PROP_OR(inst, filter_dynamic_bottom, -1),                    \
        .filter_dynamic_lower = DT_INST_PROP_OR(inst, filter_dynamic_lower, -1),                     \
        .filter_dynamic_upper = DT_INST_PROP_OR(inst, filter_dynamic_upper, -1),                     \
        .x_resolution = DT_INST_PROP_OR(inst, x_resolution, -1),                                     \
        .y_resolution = DT_INST_PROP_OR(inst, y_resolution, -1),                                     \
        .swipe_initial_distance = DT_INST_PROP_OR(inst, swipe_initial_distance, -1),                 \
        .swipe_initial_time = DT_INST_PROP_OR(inst, swipe_initial_time, -1),                         \
        .swipe_angle = DT_INST_PROP_OR(inst, swipe_angle, -1),                                       \
        .swipe_consecutive_distance = DT_INST_PROP_OR(inst, swipe_consecutive_distance, -1),         \
        .swipe_consecutive_time = DT_INST_PROP_OR(inst, swipe_consecutive_time, -1),                 \
        .scroll_initial_distance = DT_INST_PROP_OR(inst, scroll_initial_distance, -1),               \
        .scroll_angle = DT_INST_PROP_OR(inst, scroll_angle, -1),                                     \
        .zoom_initial_distance = DT_INST_PROP_OR(inst, zoom_initial_distance, -1),                   \
        .zoom_consecutive_distance = DT_INST_PROP_OR(inst, zoom_consecutive_distance, -1),           \
        .ati_target = DT_INST_PROP_OR(inst, ati_target, -1),                                         \
        .ref_drift_limit = DT_INST_PROP_OR(inst, ref_drift_limit, -1),                               \
        .reati_lower_limit = DT_INST_PROP_OR(inst, reati_lower_limit, -1),                           \
        .reati_upper_limit = DT_INST_PROP_OR(inst, reati_upper_limit, -1),                           \
        .max_count_limit = DT_INST_PROP_OR(inst, max_count_limit, -1),                               \
        .ati_retry_time = DT_INST_PROP_OR(inst, ati_retry_time, -1),                                 \
        .report_rate_active = DT_INST_PROP_OR(inst, report_rate_active, -1),                         \
        .report_rate_idle_touch = DT_INST_PROP_OR(inst, report_rate_idle_touch, -1),                 \
        .report_rate_idle = DT_INST_PROP_OR(inst, report_rate_idle, -1),                             \
        .report_rate_lp1 = DT_INST_PROP_OR(inst, report_rate_lp1, -1),                               \
        .report_rate_lp2 = DT_INST_PROP_OR(inst, report_rate_lp2, -1),                               \
        .timeout_active = DT_INST_PROP_OR(inst, timeout_active, -1),                                 \
        .timeout_idle_touch = DT_INST_PROP_OR(inst, timeout_idle_touch, -1),                         \
        .timeout_idle = DT_INST_PROP_OR(inst, timeout_idle, -1),                                     \
        .timeout_lp1 = DT_INST_PROP_OR(inst, timeout_lp1, -1),                                       \
        .ref_update_time = DT_INST_PROP_OR(inst, ref_update_time, -1),                               \
        .tap_time = DT_INST_PROP_OR(inst, tap_time, -1),                                             \
        .tap_distance = DT_INST_PROP_OR(inst, tap_distance, -1),                                     \
        .hold_time = DT_INST_PROP_OR(inst, hold_time, -1),                                           \
    };                                                                                               \
                                                                                                     \
    DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL, &tps43_##inst##_drvdata, &tps43_##inst##_config,   \
                        POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);


DT_INST_FOREACH_STATUS_OKAY(TPS43_INIT)

/**
 * @brief Public function to manage trackpad sleep mode
 *
 * This function is used by ZMK power management system (via tps43_idle_sleeper)
 * to put trackpad into sleep mode when keyboard transitions to idle/sleep state.
 *
 * @param dev Pointer to trackpad device
 * @param sleep true - enter sleep mode, false - wake up
 * @return 0 on success, negative error code on failure
 */
int tps43_set_sleep(const struct device *dev, bool sleep) {
    if (dev == NULL) {
        return -EINVAL;
    }
    return tps43_set_suspend(dev, sleep);
}

/**
 * @brief Public function to request an asynchronous full trackpad re-init
 *
 * Queues the reset work item (tps43_reset_work_handler); safe to call from
 * a ZMK behavior handler. Returns without waiting for the ~620 ms chip
 * re-initialization to run.
 *
 * @param dev Pointer to trackpad device
 * @return 0 when scheduled, -ENODEV if the driver never finished init
 */
int tps43_schedule_reset(const struct device *dev) {
    if (dev == NULL) {
        return -EINVAL;
    }

    struct tps43_drv_data *drv_data = dev->data;
    if (!drv_data->initialized) {
        return -ENODEV;
    }

    return k_work_schedule(&drv_data->reset_work, K_NO_WAIT);
}
