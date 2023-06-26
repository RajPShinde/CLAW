

    #include <linux/init.h>
    #include <linux/module.h>
    #include <linux/spi/spi.h>
    #include <linux/gpio.h>
    #include <linux/interrupt.h>
    #include <linux/clk-provider.h>
    #include <linux/clkdev.h>
    #include <linux/clk.h>

    int busnum = 0;
    int chip_select = 0;
    int gpio_int = 858;

    module_param(busnum, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    MODULE_PARM_DESC(busnum, "busnum of spi bus to use");

    module_param(gpio_int, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    MODULE_PARM_DESC(gpio_int, "linux gpio number of INT gpio");

    module_param(chip_select, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    MODULE_PARM_DESC(chip_select, "spi chip select");

    int gpio_requested = 0;

    struct spi_device *dev1;


    static void initialize_clocks(void)
    {
        struct clk *clk24;

        clk24 = clk_register_fixed_rate(NULL, "clk24", NULL,
                        CLK_SET_RATE_GATE, 8000000);
        clk_register_clkdev(clk24, NULL, "spi0.0");
    }


    static struct spi_board_info spi_device_info = {
    .modalias = "mcp2515",
    .irq = -1,
    .max_speed_hz = 10000000,
    };

    static int __init mcp2515_init(void)
    {
    int ret;
    struct spi_master *master;

    printk("mcp2515_init: init\n");

    ret = gpio_request(gpio_int, "sysfs");

    if(ret){
    printk("mcp2515_init: could not request gpio %d\n", gpio_int);
    gpio_free(gpio_int);
    return ret;
    }
    gpio_requested = 1;

    gpio_direction_input(gpio_int);

    initialize_clocks();

    ret = gpio_to_irq(gpio_int);
    printk("mcp2515_init: irq for pin %d is %d\n", gpio_int, ret);
    spi_device_info.irq = ret;
    spi_device_info.bus_num = busnum;
    spi_device_info.chip_select = chip_select;

    master = spi_busnum_to_master( spi_device_info.bus_num );
    if( !master ){
    printk("mcp2515_init: MASTER not found.\n");
    ret = -ENODEV;
    goto error_postgpio;
    }

    dev1 = spi_new_device( master, &spi_device_info );

    if( !dev1) {
    printk("mcp2515_init: FAILED to create slave.\n");
    ret = -ENODEV;
    goto error_postgpio;
    }

    printk("mcp2515_init: device created!\n");

    return 0;

    error_postgpio:
    gpio_free(gpio_int);
    return ret;
    }

    static void __exit mcp2515_exit(void)
    {
    printk("mcp2515_init: exit\n");

    if( dev1 ){
    spi_unregister_device(dev1);
    }
    if(gpio_requested)
    gpio_free(gpio_int);
    }
    module_init(mcp2515_init);
    module_exit(mcp2515_exit);
    MODULE_LICENSE("GPL");
    MODULE_AUTHOR("RajPShinde <[rajprakashshinde07@gmail.com]>");
    MODULE_DESCRIPTION("MCP2515 init");
