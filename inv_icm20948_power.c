// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Bharadwaj Raju <bharadwaj.raju777@gmail.com>
 */

#include "inv_icm20948.h"

static int inv_icm20948_suspend(struct device *dev)
{
	struct inv_icm20948_state *state;
	int ret;

	if (pm_runtime_suspended(dev))
		return 0;

	state = dev_get_drvdata(dev);

	mutex_lock(&state->lock);

	ret = regmap_write_bits(state->regmap, INV_ICM20948_REG_PWR_MGMT_1,
				INV_ICM20948_PWR_MGMT_1_SLEEP,
				INV_ICM20948_PWR_MGMT_1_SLEEP);
	mutex_unlock(&state->lock);
	return ret;
}

static int inv_icm20948_resume(struct device *dev)
{
	struct inv_icm20948_state *state = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&state->lock);

	pm_runtime_disable(state->dev);
	pm_runtime_set_active(state->dev);
	pm_runtime_enable(state->dev);

	ret = regmap_write_bits(state->regmap, INV_ICM20948_REG_PWR_MGMT_1,
				INV_ICM20948_PWR_MGMT_1_SLEEP, 0);
	if (ret)
		goto out;

	msleep(INV_ICM20948_SLEEP_WAKEUP_MS);

	ret = 0;

out:
	mutex_unlock(&state->lock);
	return ret;
}

static void inv_icm20948_pm_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

int inv_icm20948_pm_setup(struct inv_icm20948_state *state)
{
	struct device *dev = state->dev;

	int ret;

	mutex_lock(&state->lock);
	ret = pm_runtime_set_active(dev);
	if (ret)
		goto out;
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, INV_ICM20948_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put(dev);

	ret = devm_add_action_or_reset(dev, inv_icm20948_pm_disable, dev);

out:
	mutex_unlock(&state->lock);
	return ret;
}

const struct dev_pm_ops inv_icm20948_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_icm20948_suspend, inv_icm20948_resume)
	SET_RUNTIME_PM_OPS(inv_icm20948_suspend, inv_icm20948_resume, NULL)
};
