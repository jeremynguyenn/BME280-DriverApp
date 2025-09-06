/**
 * @brief I2C driver for Bosch Sensortec BME280 combined temperature, pressure,
 * humidity sensor
 *
 * @author Nguyen Nhan
 * @version 1.0
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <module.h>
#include <bme280.h>
#include <bme280_regs_mapp.h>
#include <bme280_info_mapp.h>

/** Character device variables */
static dev_t bme280_dev;
static struct cdev bme280_cdev;
static struct class *bme280_class;
static const char *bme280_dev_name = "bme280";
static int bme280_major;

/** Pointer to last selected device, all operations performed with this device */
struct bme280 *bme280_device = NULL;

/** Linked list to hold registered devices */
LIST_HEAD(bme280_devices);

/** Used for guaranteed sole access to registered devices for this driver */
struct mutex bme280_devices_lock;

/** GPIO and Interrupt variables */
static unsigned int gpioButton = 24;
static unsigned int gpioLED = 23;
static int irqNumber;
int period_ms = 2000;
static int button_pressed = 0;
static struct timespec ts_last, ts_current, ts_diff;
static const int period_start_ms = 2000;
static const int period_divisor = 2;
static const int max_step_count = 4;

/** Interrupt handler function */
static irq_handler_t period_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
    getnstimeofday(&ts_current);
    ts_diff = timespec_sub(ts_current, ts_last);
    if ((ts_diff.tv_sec * 1000 + ts_diff.tv_nsec / 1000000) < 250) {
        return (irq_handler_t) IRQ_NONE;
    }
    ts_last = ts_current;
    button_pressed++;
    if (button_pressed % max_step_count) {
        period_ms /= period_divisor;
    } else {
        period_ms = period_start_ms;
        button_pressed = 0;  // Reset counter
    }
    return (irq_handler_t) IRQ_HANDLED;
}

/** Character device file operations */
static int bme280_dev_open(struct inode *inode, struct file *file)
{
    mutex_lock(&bme280_devices_lock);
    if (bme280_device == NULL) {
        pr_err(THIS_MODULE_NAME ": no device selected, use /sys/bme280/i2c\n");
        mutex_unlock(&bme280_devices_lock);
        return -ENODEV;
    }
    mutex_unlock(&bme280_devices_lock);
    return 0;
}

static ssize_t bme280_dev_read(struct file *file, char __user *buf, size_t count, loff_t *off)
{
    struct bme280_data comp_data;
    char data_buf[128];
    size_t data_len;
    ssize_t ret;

    mutex_lock(&bme280_devices_lock);

    if (bme280_device == NULL) {
        pr_err(THIS_MODULE_NAME ": no device selected\n");
        ret = -ENODEV;
        goto err;
    }

    ret = bme280_get_sensor_data_forced(bme280_device, BME280_ALL, &comp_data);
    if (ret != BME280_OK) {
        pr_err(THIS_MODULE_NAME ": failed to read sensor data\n");
        goto err;
    }

    data_len = snprintf(data_buf, sizeof(data_buf),
                        "pressure: %d Pa\ntemperature: %d m°C\nhumidity: %d%%\n",
                        comp_data.pressure, comp_data.temperature, comp_data.humidity / 1024);
    if (data_len >= sizeof(data_buf)) {
        pr_err(THIS_MODULE_NAME ": data buffer overflow\n");
        ret = -EOVERFLOW;
        goto err;
    }

    if (*off >= data_len) {
        ret = 0;
        goto err;
    }

    if (count > data_len - *off) {
        count = data_len - *off;
    }

    if (copy_to_user(buf, data_buf + *off, count)) {
        pr_err(THIS_MODULE_NAME ": failed to copy data to user\n");
        ret = -EFAULT;
        goto err;
    }

    *off += count;
    ret = count;

err:
    mutex_unlock(&bme280_devices_lock);
    return ret;
}

static int bme280_dev_release(struct inode *inode, struct file *file)
{
    return 0;
}

static const struct file_operations bme280_dev_fops = {
    .owner = THIS_MODULE,
    .open = bme280_dev_open,
    .read = bme280_dev_read,
    .release = bme280_dev_release,
};

/** Register and initialize new device */
static ssize_t bme280_i2c_register_device(struct i2c_client *client)
{
    ssize_t ret;

    struct bme280 *device = kzalloc(sizeof(*device), GFP_KERNEL);
    if (device == NULL) {
        pr_err(THIS_MODULE_NAME
               ": failed to allocate memory for device at "
               " %s-%d 0x%x\n",
               client->adapter->dev.of_node->name, client->adapter->nr,
               client->addr);
        ret = -ENOMEM;
        goto err;
    }

    ret = bme280_init(device, client);
    if (ret != BME280_OK) {
        pr_err(THIS_MODULE_NAME
               ": failed to initialize device at %s-%d 0x%x\n",
               client->adapter->dev.of_node->name, client->adapter->nr,
               client->addr);
        goto cleanup_device;
    }

    /** Default settings for new device */
    device->settings.osrs_p = BME280_INDOOR_PRESS_OVERSAMPLING;
    device->settings.osrs_t = BME280_INDOOR_TEMP_OVERSAMPLING;
    device->settings.osrs_h = BME280_INDOOR_HUM_OVERSAMPLING;
    device->settings.filter = BME280_INDOOR_FILTER_COEFF;
    device->settings.standby_time = BME280_INDOOR_STANDBY_TIME;

    ret = bme280_set_sensor_settings(device, BME280_ALL_SETTINGS_SEL);
    if (ret != BME280_OK) {
        pr_err(THIS_MODULE_NAME
               ": failed to set default settings for device at %s-%d 0x%x\n",
               client->adapter->dev.of_node->name, client->adapter->nr,
               client->addr);
        goto cleanup_device;
    }

    mutex_lock(&bme280_devices_lock);
    list_add_tail(&device->list, &bme280_devices);
    bme280_device = device; /* Set last device as current */
    mutex_unlock(&bme280_devices_lock);

    return 0;

cleanup_device:
    kfree(device);
err:
    return ret;
}

/** Unregister and free device */
static ssize_t bme280_i2c_unregister_device(struct i2c_client *client)
{
    struct bme280 *pos, *n;

    mutex_lock(&bme280_devices_lock);

    list_for_each_entry_safe(pos, n, &bme280_devices, list) {
        if (pos->client == client) {
            if (bme280_device == pos) {
                if (list_is_last(&pos->list, &bme280_devices)) {
                    bme280_device = NULL;
                } else {
                    bme280_device = list_next_entry(pos, list);
                }
            }

            list_del(&pos->list);
            kfree(pos);

            mutex_unlock(&bme280_devices_lock);
            return 0;
        }
    }

    mutex_unlock(&bme280_devices_lock);
    return -ENODEV;
}

static int bme280_i2c_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
    int ret;

    mutex_init(&bme280_devices_lock);

    ret = gpio_request(gpioLED, "bme280_led");
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to request LED GPIO %d\n", gpioLED);
        goto err;
    }

    ret = gpio_direction_output(gpioLED, 0);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to set LED GPIO %d direction\n", gpioLED);
        goto free_gpio_led;
    }

    ret = gpio_export(gpioLED, false);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to export LED GPIO %d\n", gpioLED);
        goto free_gpio_led;
    }

    ret = gpio_request(gpioButton, "bme280_button");
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to request Button GPIO %d\n", gpioButton);
        goto unexport_gpio_led;
    }

    ret = gpio_direction_input(gpioButton);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to set Button GPIO %d direction\n", gpioButton);
        goto free_gpio_button;
    }

    ret = gpio_export(gpioButton, false);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to export Button GPIO %d\n", gpioButton);
        goto free_gpio_button;
    }

    irqNumber = gpio_to_irq(gpioButton);
    if (irqNumber < 0) {
        pr_err(THIS_MODULE_NAME ": failed to get IRQ for Button GPIO %d\n", gpioButton);
        ret = irqNumber;
        goto unexport_gpio_button;
    }

    ret = request_irq(irqNumber, (irq_handler_t)period_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      "bme280_button_handler", NULL);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to request IRQ %d\n", irqNumber);
        goto unexport_gpio_button;
    }

    getnstimeofday(&ts_last);

    ret = bme280_i2c_register_device(client);
    if (ret) {
        pr_err(THIS_MODULE_NAME
               ": failed to register device at %s-%d 0x%x\n",
               client->adapter->dev.of_node->name,
               client->adapter->nr, client->addr);
        goto free_irq;
    }

    ret = bme280_create_info_mapp();
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to create procfs mappings\n");
        goto unregister_device;
    }

    ret = bme280_create_regs_mapp();
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to create sysfs mappings\n");
        goto remove_info_mapp;
    }

    /** Initialize character device */
    ret = alloc_chrdev_region(&bme280_dev, 0, 1, bme280_dev_name);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to allocate char device region\n");
        goto remove_regs_mapp;
    }

    bme280_major = MAJOR(bme280_dev);
    cdev_init(&bme280_cdev, &bme280_dev_fops);
    ret = cdev_add(&bme280_cdev, bme280_dev, 1);
    if (ret) {
        pr_err(THIS_MODULE_NAME ": failed to add char device\n");
        goto unregister_chrdev;
    }

    bme280_class = class_create(THIS_MODULE, bme280_dev_name);
    if (IS_ERR(bme280_class)) {
        pr_err(THIS_MODULE_NAME ": failed to create device class\n");
        ret = PTR_ERR(bme280_class);
        goto cdev_del;
    }

    if (!device_create(bme280_class, NULL, bme280_dev, NULL, bme280_dev_name)) {
        pr_err(THIS_MODULE_NAME ": failed to create device\n");
        ret = -ENODEV;
        goto class_destroy;
    }

    pr_info(THIS_MODULE_NAME ": registered device at %s-%d 0x%x, char device /dev/%s\n",
            client->adapter->dev.of_node->name, client->adapter->nr,
            client->addr, bme280_dev_name);

    return 0;

class_destroy:
    class_destroy(bme280_class);
cdev_del:
    cdev_del(&bme280_cdev);
unregister_chrdev:
    unregister_chrdev_region(bme280_dev, 1);
remove_regs_mapp:
    bme280_remove_regs_mapp();
remove_info_mapp:
    bme280_remove_info_mapp();
unregister_device:
    bme280_i2c_unregister_device(client);
free_irq:
    free_irq(irqNumber, NULL);
unexport_gpio_button:
    gpio_unexport(gpioButton);
free_gpio_button:
    gpio_free(gpioButton);
unexport_gpio_led:
    gpio_unexport(gpioLED);
free_gpio_led:
    gpio_free(gpioLED);
err:
    mutex_destroy(&bme280_devices_lock);
    return ret;
}

static int bme280_i2c_remove(struct i2c_client *client)
{
    int ret;

    device_destroy(bme280_class, bme280_dev);
    class_destroy(bme280_class);
    cdev_del(&bme280_cdev);
    unregister_chrdev_region(bme280_dev, 1);

    bme280_remove_regs_mapp();
    bme280_remove_info_mapp();

    ret = bme280_i2c_unregister_device(client);
    if (ret) {
        pr_err(THIS_MODULE_NAME
               ": failed to unregister device at %s-%d 0x%x\n",
               client->adapter->dev.of_node->name,
               client->adapter->nr, client->addr);
    } else {
        pr_info(THIS_MODULE_NAME ": unregistered device at %s-%d 0x%x\n",
                client->adapter->dev.of_node->name, client->adapter->nr,
                client->addr);
    }

    free_irq(irqNumber, NULL);
    gpio_set_value(gpioLED, 0);
    gpio_unexport(gpioLED);
    gpio_unexport(gpioButton);
    gpio_free(gpioLED);
    gpio_free(gpioButton);

    mutex_destroy(&bme280_devices_lock);

    return ret;
}

static const unsigned short bme280_i2c_addr[] = { BME280_I2C_ADDR_PRIM,
                                                  BME280_I2C_ADDR_SEC,
                                                  I2C_CLIENT_END };
static const struct i2c_device_id bme280_i2c_id[] = { { "bme280",
                                                        BME280_CHIP_ID },
                                                      {} };
MODULE_DEVICE_TABLE(i2c, bme280_i2c_id);

static struct i2c_driver bme280_i2c_driver = {
    .driver = {
        .name   = "bme280",
    },
    .probe = bme280_i2c_probe,
    .remove = bme280_i2c_remove,
    .id_table = bme280_i2c_id,
    .address_list = bme280_i2c_addr,
};
module_i2c_driver(bme280_i2c_driver);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Driver for Bosch Sensortec BME280 combined "
                   "temperature, pressure, humidity sensor");
MODULE_VERSION("1.0");