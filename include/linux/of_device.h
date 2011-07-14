#ifndef _LINUX_OF_DEVICE_H
#define _LINUX_OF_DEVICE_H

#ifdef CONFIG_OF_DEVICE
#include <linux/device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>

#include <asm/of_device.h>

#define	to_of_device(d) container_of(d, struct of_device, dev)

extern const struct of_device_id *of_match_device(
	const struct of_device_id *matches, const struct device *dev);

extern void of_device_make_bus_id(struct device *dev);

/**
 * of_driver_match_device - Tell if a driver's of_match_table matches a device.
 * @drv: the device_driver structure to test
 * @dev: the device structure to match against
 */
static inline int of_driver_match_device(const struct device *dev,
                                         const struct device_driver *drv)
{
        return of_match_device(drv->of_match_table, dev) != NULL;
}

extern struct of_device *of_dev_get(struct of_device *dev);
extern void of_dev_put(struct of_device *dev);

extern int of_device_register(struct of_device *ofdev);
extern void of_device_unregister(struct of_device *ofdev);
extern void of_release_dev(struct device *dev);

static inline void of_device_free(struct of_device *dev)
{
	of_release_dev(&dev->dev);
}

extern ssize_t of_device_get_modalias(struct of_device *ofdev,
					char *str, ssize_t len);
#endif /* CONFIG_OF_DEVICE */

static inline int of_driver_match_device(struct device *dev,
                                         struct device_driver *drv)
{
        return 0;
}

#endif /* _LINUX_OF_DEVICE_H */
