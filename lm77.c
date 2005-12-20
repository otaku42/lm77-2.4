/*
    lm77.c - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Copyright (c) 2005  Michael Renzmann <mrenzmann@otaku42.de>
    
    Heavily based on lm77.c for kernel 2.6.x, which is:
    Copyright (c) 2004  Andras BALI <drewie@freemail.hu>
    
    Andras' work in return is heavily based on lm75.c by 
    Frodo Looijaard <frodol@dds.nl>. 
    
    The LM77 is a temperature sensor and thermal window comparator with
    0.5 deg resolution made by National Semiconductor. Complete datasheet
    can be obtained at their site:
       http://national.com/pf/LM/LM77.html

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-proc.h>
#include <linux/init.h>
#include "version.h"
#include "lm77.h"

/* Addresses to scan */
static unsigned short normal_i2c[] = { SENSORS_I2C_END };
static unsigned short normal_i2c_range[] = { 0x48, 0x4b, SENSORS_I2C_END };
static unsigned int normal_isa[] = { SENSORS_ISA_END };
static unsigned int normal_isa_range[] = { SENSORS_ISA_END };

/* Insmod parameters */
SENSORS_INSMOD_1(lm77);

/* Whether or not to compile with debugging extensions */
#define DEBUG 1
/* #undef DEBUG */

/* The LM77 registers */
#define LM77_REG_TEMP 0x00		/* Current temperature (read-only) */
#define LM77_REG_CONF 0x01		/* Configuration (read-write) */
#define LM77_REG_T_HYST 0x02		/* Hysteresis (read-write) */
#define LM77_REG_T_CRIT 0x03		/* Critical temperature (read-write) */
#define LM77_REG_T_LOW 0x04		/* Minimum temperature (read-write) */
#define LM77_REG_T_HIGH 0x05		/* Maximum temperature (read-write) */

/* LM77 configuration bits (for LM77_REG_CONF) */
#define LM77_CONF_SHUTDOWN 0x1		/* Shutdown */
#define LM77_CONF_INTMODE 0x2		/* Interrupt mode */
#define LM77_CONF_TCRITPOL 0x4		/* T_CRIT_A polarity */
#define LM77_CONF_INTPOL 0x8		/* INT polarity */
#define LM77_CONF_FAULTQ 0x10		/* Fault queue */

/* LM77 default register values */
#ifdef DEBUG
#define LM77_DEFAULT_CONF 0x0
#define LM77_DEFAULT_T_LOW 0xa0		/* 10 deg celcius */
#define LM77_DEFAULT_T_HIGH 0x400	/* 64 deg celcius */
#define LM77_DEFAULT_T_CRIT 0x500	/* 80 deg celcius */
#define LM77_DEFAULT_T_HYST 0x20	/* 2 deg celcius */
#endif

/* Misc. defines */

/* LM77_SC_NOTSET is used to signal that a value of the array that is
 * passed to the sanity check routine should be regarded as "not given
 * by the user"
 */
#define LM77_SC_NOTSET -99999

/* mask for the alarm bits in LM77_REG_TEMP */
#define LM77_ALARM_MASK 0x0007


/* Each client has this additional data */
struct lm77_data {
	struct i2c_client client;
	int sysctl_id;

	struct semaphore update_lock;
	char valid;
	unsigned long last_updated;	/* In jiffies */

	int temp_input;			/* Current temperature */
	int temp_crit; 			/* Critical temperature bound */
	int temp_min;			/* Minimum temperature bound */
	int temp_max;			/* Maximum temperature bound */
	int temp_hyst;			/* Hysteresis (relative) */
	u8 alarms;			/* Alarm bits */
};

static int lm77_attach_adapter(struct i2c_adapter *adapter);
static int lm77_detect(struct i2c_adapter *adapter, int address,
		       unsigned short flags, int kind);
static void lm77_init_client(struct i2c_client *client);
static int lm77_detach_client(struct i2c_client *client);

static int lm77_read_value(struct i2c_client *client, u8 reg);
static int lm77_write_value(struct i2c_client *client, u8 reg, u16 value);
static void lm77_update_client(struct i2c_client *client);

static void lm77_proc_temp(struct i2c_client *client, int operation,
		      int ctl_name, int *nrels_mag, long *results);
static void lm77_proc_alarms(struct i2c_client *client, int operation,
		      int ctl_name, int *nrels_mag, long *results);
#ifdef DEBUG
static void lm77_proc_reset(struct i2c_client *client, int operation,
		      int ctl_name, int *nrels_mag, long *results);
#endif

/* This is the driver that will be inserted */
static struct i2c_driver lm77_driver = {
	.name		= "LM77 sensor chip driver",
	.flags		= I2C_DF_NOTIFY,
	.attach_adapter	= lm77_attach_adapter,
	.detach_client	= lm77_detach_client,
};

/* -- SENSORS SYSCTL START -- */

#define LM77_SYSCTL_TEMP 1200 		/* Temperature readings */
#define LM77_SYSCTL_TEMP_CRIT 1201	/* Critical temperature bound */
#define LM77_SYSCTL_TEMP_HYST 1202	/* Hysteresis */
#define LM77_SYSCTL_ALARMS 1203		/* Current alarm status */
#ifdef DEBUG
#define LM77_SYSCTL_RESET 1204		/* LM77 reset */
#endif

/* -- SENSORS SYSCTL END -- */

/* These files are created for each detected LM7/. This is just a template;
   though at first sight, you might think we could use a statically
   allocated list, we need some way to get back to the parent - which
   is done through one of the 'extra' fields which are initialized
   when a new copy is allocated. */
static ctl_table lm77_dir_table_template[] = {
	{LM77_SYSCTL_TEMP, "temp", NULL, 0, 0644, NULL, &i2c_proc_real,
	 &i2c_sysctl_real, NULL, &lm77_proc_temp},
	{LM77_SYSCTL_TEMP_CRIT, "temp_crit", NULL, 0, 0644, NULL, &i2c_proc_real,
	 &i2c_sysctl_real, NULL, &lm77_proc_temp},
	{LM77_SYSCTL_TEMP_HYST, "temp_hyst", NULL, 0, 0644, NULL, &i2c_proc_real,
	 &i2c_sysctl_real, NULL, &lm77_proc_temp},
	{LM77_SYSCTL_ALARMS, "alarms", NULL, 0, 0644, NULL, &i2c_proc_real,
	 &i2c_sysctl_real, NULL, &lm77_proc_alarms},
#ifdef DEBUG
	{LM77_SYSCTL_RESET, "reset", NULL, 0, 0644, NULL, &i2c_proc_real,
	 &i2c_sysctl_real, NULL, &lm77_proc_reset},
#endif
	{0}
};



static int lm77_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_detect(adapter, &addr_data, lm77_detect);
}

/* This function is called by i2c_detect */
int lm77_detect(struct i2c_adapter *adapter, int address,
		unsigned short flags, int kind)
{
	int i;
	struct i2c_client *new_client;
	struct lm77_data *data;
	int err = 0;
	const char *type_name, *client_name;

	/* Make sure we aren't probing the ISA bus!! This is just a safety check
	   at this moment; i2c_detect really won't call us. */
#ifdef DEBUG
	if (i2c_is_isa_adapter(adapter)) {
		printk
		    ("lm77.o: lm77_detect called for an ISA bus adapter?!?\n");
		return 0;
	}
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		    goto error0;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet.
	   But it allows us to access lm75_{read,write}_value. */
	if (!(data = kmalloc(sizeof(struct lm77_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto error0;
	}
	memset(data, 0, sizeof(struct lm77_data));

	new_client = &data->client;
	new_client->addr = address;
	new_client->data = data;
	new_client->adapter = adapter;
	new_client->driver = &lm77_driver;
	new_client->flags = 0;

	/* Here comes the remaining detection.  Since the LM77 has no
	   registers dedicated to identification, we have to rely on the
	   following tricks:
	   
	   1. the high 4 bits represent the sign and thus they should
	      always be the same
	   2. the high 3 bits are unused in the configuration register
	   3. addresses 0x06 and 0x07 return the last read value
	   4. registers cycling over 8-address boundaries
	   
	   Word-sized registers are high-byte first. */
	if (kind < 0) {
		int i, cur, conf, hyst, crit, min, max;

		/* Unused addresses */
		cur = i2c_smbus_read_word_data(new_client, 0);
		conf = i2c_smbus_read_byte_data(new_client, 1);
		hyst = i2c_smbus_read_word_data(new_client, 2);
		crit = i2c_smbus_read_word_data(new_client, 3);
		min = i2c_smbus_read_word_data(new_client, 4);
		max = i2c_smbus_read_word_data(new_client, 5);
		for (i = 8; i <= 0xff; i += 8)
			if (i2c_smbus_read_byte_data(new_client, i + 1) != conf
			    || i2c_smbus_read_word_data(new_client, i + 2) != hyst
			    || i2c_smbus_read_word_data(new_client, i + 3) != crit
			    || i2c_smbus_read_word_data(new_client, i + 4) != min
			    || i2c_smbus_read_word_data(new_client, i + 5) != max)
				goto error1;


		/* sign bits */
		if (((cur & 0x00f0) != 0xf0 && (cur & 0x00f0) != 0x0)
		    || ((hyst & 0x00f0) != 0xf0 && (hyst & 0x00f0) != 0x0)
		    || ((crit & 0x00f0) != 0xf0 && (crit & 0x00f0) != 0x0)
		    || ((min & 0x00f0) != 0xf0 && (min & 0x00f0) != 0x0)
		    || ((max & 0x00f0) != 0xf0 && (max & 0x00f0) != 0x0))
			goto error1;

		/* unused bits */
		if (conf & 0xe0)
		 	goto error1;

		/* 0x06 and 0x07 return last read value */
		cur = i2c_smbus_read_word_data(new_client, 0);
		if (i2c_smbus_read_word_data(new_client, 6) != cur
		    || i2c_smbus_read_word_data(new_client, 7) != cur)
			goto error1;
		hyst = i2c_smbus_read_word_data(new_client, 0);
		if (i2c_smbus_read_word_data(new_client, 6) != hyst
		    || i2c_smbus_read_word_data(new_client, 7) != hyst)
			goto error1;
		hyst = i2c_smbus_read_word_data(new_client, 0);
		if (i2c_smbus_read_word_data(new_client, 6) != hyst
		    || i2c_smbus_read_word_data(new_client, 7) != hyst)
			goto error1;
	}

	/* Determine the chip type - only one kind supported! */
	if (kind <= 0)
		kind = lm77;

	if (kind == lm77) {
		type_name = "lm77";
		client_name = "LM77 chip";
	} else {
		pr_debug("lm77.o: Internal error: unknown kind (%d)?!?", kind);
		goto error1;
	}

	/* Fill in the remaining client fields and put it into the global list */
	strcpy(new_client->name, client_name);
	data->valid = 0;
	init_MUTEX(&data->update_lock);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto error3;

	/* Register a new directory entry with module sensors */
	if ((i = i2c_register_entry(new_client, type_name,
					lm77_dir_table_template,
					THIS_MODULE)) < 0) {
		err = i;
		goto error4;
	}
	data->sysctl_id = i;

	/* Initialize the LM77 chip */
	lm77_init_client(new_client);
	return 0;

/* OK, this is not exactly good programming practice, usually. But it is
   very code-efficient in this case. */

      error4:
	i2c_detach_client(new_client);
      error3:
      error1:
	kfree(data);
      error0:
	return err;
}

static void lm77_init_client(struct i2c_client *client)
{
	/* Initialize the LM77 chip - turn off shutdown mode */
	u16 conf = lm77_read_value(client, LM77_REG_CONF);
	u16 new = 0;
	
	if (conf & LM77_CONF_SHUTDOWN) {
		printk(KERN_INFO "lm77: waking up bus %d, io %x\n",
			client->adapter->id, client->addr);
		/* this is done automatically with the default for new */
	}

#ifdef LM77_FAULT_QUEUE
	/* Define this at compile time in order to enable the fault queue
	 * feature provided by the LM77. Refer to section 1.7 of the LM77
	 * data sheet for further information.
	 */
	conf |= LM77_CONF_FAULTQ
#endif

	lm77_write_value(client, LM77_REG_CONF, new);	
}

static int lm77_detach_client(struct i2c_client *client)
{
	struct lm77_data *data = client->data;

	i2c_deregister_entry(data->sysctl_id);
	i2c_detach_client(client);
	kfree(client->data);
	return 0;
}



/* All registers are word-sized, except for the configuration register.
   The LM77 uses a high-byte first convention, which is exactly opposite to
   the usual practice. */
static int lm77_read_value(struct i2c_client *client, u8 reg)
{
	if (reg == LM77_REG_CONF)
		return i2c_smbus_read_byte_data(client, reg);
	else
		return swab16(i2c_smbus_read_word_data(client, reg));
}

/* All registers are word-sized, except for the configuration register.
   The LM77 uses a high-byte first convention, which is exactly opposite to
   the usual practice. */
static int lm77_write_value(struct i2c_client *client, u8 reg, u16 value)
{
	if (reg == LM77_REG_CONF)
		return i2c_smbus_write_byte_data(client, reg, value);
	else
		return i2c_smbus_write_word_data(client, reg, swab16(value));
}

static void lm77_update_client(struct i2c_client *client)
{
	struct lm77_data *data = client->data;

	down(&data->update_lock);

	if ((jiffies - data->last_updated > HZ + HZ / 2) ||
	    (jiffies < data->last_updated) || !data->valid) {

		pr_debug("Starting lm77 update\n");

		data->temp_input =
		    LM77_TEMP_FROM_REG(lm77_read_value(client,
		                                       LM77_REG_TEMP));
		data->temp_hyst =
		    LM77_TEMP_FROM_REG(lm77_read_value(client,
		                                       LM77_REG_T_HYST));
		data->temp_crit =
		    LM77_TEMP_FROM_REG(lm77_read_value(client,
		                                       LM77_REG_T_CRIT));
		data->temp_min =
		    LM77_TEMP_FROM_REG(lm77_read_value(client,
		                                       LM77_REG_T_LOW));
		data->temp_max =
		    LM77_TEMP_FROM_REG(lm77_read_value(client,
		                                       LM77_REG_T_HIGH));
		data->alarms =
		    lm77_read_value(client, LM77_REG_TEMP) & LM77_ALARM_MASK;
		    
		data->last_updated = jiffies;
		data->valid = 1;
		
	}

	up(&data->update_lock);
}

void lm77_proc_temp(struct i2c_client *client, int operation, int ctl_name,
	       int *nrels_mag, long *results)
{
	struct lm77_data *data = client->data;
	int new[4] = { LM77_SC_NOTSET, LM77_SC_NOTSET,
		       LM77_SC_NOTSET, LM77_SC_NOTSET };
	int check[4];
	int passed = 1;

	if (operation == SENSORS_PROC_REAL_INFO)
		*nrels_mag = 1;
	else if (operation == SENSORS_PROC_REAL_READ) {
		lm77_update_client(client);
		
		switch(ctl_name) {
		case LM77_SYSCTL_TEMP:
			results[0] = data->temp_min;
			results[1] = data->temp_max;
			results[2] = data->temp_input;
			*nrels_mag = 3;
			break;
			
		case LM77_SYSCTL_TEMP_CRIT:
			results[0] = data->temp_crit;
			*nrels_mag = 1;
			break;
		
		case LM77_SYSCTL_TEMP_HYST:
			results[0] = data->temp_hyst;
			*nrels_mag = 1;
			break;
		}
	} else if (operation == SENSORS_PROC_REAL_WRITE) {
		switch(ctl_name) {
		case LM77_SYSCTL_TEMP:
			if (*nrels_mag >= 1)
				new[0] = results[0];
			if (*nrels_mag >= 2)
				new[1] = results[1];
			break;
		
		case LM77_SYSCTL_TEMP_CRIT:
			if (*nrels_mag >= 1)
				new[2] = results[0];
			break;
		
		case LM77_SYSCTL_TEMP_HYST:
			if (*nrels_mag >= 1)
				new[3] = results[0];
			break;
		}

		/* populate check array; use current values where
		 * user didn't specify something else.
		 *
		 * mapping (also valid for new[]):
		 * check[0] = temp_min
		 * check[1] = temp_max
		 * check[2] = temp_crit
		 * check[3] = temp_hyst
		 */
		check[0] = (new[0] == LM77_SC_NOTSET) ? data->temp_min : new[0];
		check[1] = (new[1] == LM77_SC_NOTSET) ? data->temp_max : new[1];
		check[2] = (new[2] == LM77_SC_NOTSET) ? data->temp_crit : new[2];
		check[3] = (new[3] == LM77_SC_NOTSET) ? data->temp_hyst : new[3];

		/* check 1: LM77_TEMP_MIN >= temp_min <= LM77_TEMP_MAX */
		if ((check[0] < LM77_TEMP_MIN) ||
		    (check[0] > LM77_TEMP_MAX)) {
			printk(KERN_NOTICE "lm77: error: temp_min out of bounds\n");
			passed = 0;
		}

		/* check 2: LM77_TEMP_MIN >= temp_max <= LM77_TEMP_MAX */
		if ((check[1] < LM77_TEMP_MIN) ||
		    (check[1] > LM77_TEMP_MAX)) {
			printk(KERN_NOTICE "lm77: error: temp_max out of bounds\n");
			passed = 0;
		}
		
		/* check 3: temp_min < temp_max */
		if (check[0] >= check[1]) {
			printk(KERN_NOTICE "lm77: error; temp_min > temp_max\n");
			passed = 0;
		}

		/* 4. check[0] + check[4] < check[1] - check[4]
		 *    (see data sheet, sect. 1.1.2, page 8)
		 */
		if ((check[0] + check[4]) >= (check[1] - check[4])) {
			printk(KERN_NOTICE "lm77: error: overlapping setpoints\n");
			passed = 0;
		}

		/* apply changes only if checks pass */
		if (passed) {
			down(&data->update_lock);

			if (new[0] != LM77_SC_NOTSET) {
				data->temp_min = new[0];
				lm77_write_value(client, LM77_REG_T_LOW, LM77_TEMP_TO_REG(new[0]));
			}
			if (new[1] != LM77_SC_NOTSET) {
				data->temp_max = new[1];
				lm77_write_value(client, LM77_REG_T_HIGH, LM77_TEMP_TO_REG(new[1]));
			}
			if (new[2] != LM77_SC_NOTSET) {
				data->temp_crit = new[2];
				lm77_write_value(client, LM77_REG_T_CRIT, LM77_TEMP_TO_REG(new[2]));
			}
			if (new[3] != LM77_SC_NOTSET) {
				data->temp_max = new[3];
				lm77_write_value(client, LM77_REG_T_HYST, LM77_TEMP_TO_REG(new[3]));
			}

			printk(KERN_INFO "lm77: changes applied.\n");
			
			up(&data->update_lock);
		} else
			printk(KERN_NOTICE "lm77: changes not applied.\n");
	}
}

void lm77_proc_alarms(struct i2c_client *client, int operation, int ctl_name,
		 int *nrels_mag, long *results)
{
	struct lm77_data *data = client->data;
	
	if (operation == SENSORS_PROC_REAL_INFO)
		*nrels_mag = 0;
	else if (operation == SENSORS_PROC_REAL_READ) {
		lm77_update_client(client);
		results[0] = data->alarms;
		*nrels_mag = 1;
	} else if (operation == SENSORS_PROC_REAL_WRITE) {
		printk(KERN_NOTICE "lm77: alarms: read-only\n");
		*nrels_mag = 0;
	}
}

#ifdef DEBUG
void lm77_proc_reset(struct i2c_client *client, int operation, int ctl_name,
		 int *nrels_mag, long *results)
{
	if (operation == SENSORS_PROC_REAL_INFO)
		*nrels_mag = 0;
	else if (operation == SENSORS_PROC_REAL_READ)
		/* nothing to read */
		*nrels_mag = 0;
	else if (operation == SENSORS_PROC_REAL_WRITE) {
		if ((*nrels_mag >= 1) && (results[0] == 1)) {
			/* avoid using LM77_TEMP_TO_REG et al here, so that resetting
			 * works even when the conversion is broken
			*/
			lm77_write_value(client, LM77_REG_CONF, LM77_DEFAULT_CONF);
			lm77_write_value(client, LM77_REG_T_LOW, LM77_DEFAULT_T_LOW);
			lm77_write_value(client, LM77_REG_T_HIGH, LM77_DEFAULT_T_HIGH);
			lm77_write_value(client, LM77_REG_T_CRIT, LM77_DEFAULT_T_CRIT);
			lm77_write_value(client, LM77_REG_T_HYST, LM77_DEFAULT_T_HYST);

			printk(KERN_NOTICE "lm77: registers reset to their default,\n");
		}
	}
}
#endif

static int __init sm_lm77_init(void)
{
#ifdef DEBUG
	printk(KERN_INFO "lm77.o version %s (%s+debug)\n", LM_VERSION, LM_DATE);
#else
	printk(KERN_INFO "lm77.o version %s (%s)\n", LM_VERSION, LM_DATE);
#endif	
	return i2c_add_driver(&lm77_driver);
}

static void __exit sm_lm77_exit(void)
{
	i2c_del_driver(&lm77_driver);
}

MODULE_AUTHOR("Michael Renzmann <mrenzmann@otaku42.de>");
MODULE_DESCRIPTION("LM77 driver");
MODULE_LICENSE("GPL");

module_init(sm_lm77_init);
module_exit(sm_lm77_exit);
