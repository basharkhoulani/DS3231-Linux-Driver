/*******************************************************************************
* Institut für Rechnerarchitektur und Systemprogrammierung *
* Universität Kassel *
*******************************************************************************
* Benutzeraccount : sysprog-11 *
*******************************************************************************
* Author-1 : Bashar Khoulani
* Matrikelnummer-1 : 35831929

*******************************************************************************
* Author-2 : Adnan Noah Nezovic
* Matrikelnummer-2 : 35824000
*******************************************************************************
* Beschreibung: Linux Treiber für RTC-DS3231
* Version: 1.0
******************************************************************************/

/*
* @file ds3231_drv.c
* @author Bashar Khoulani
* @author Adnan Noah Nezovic
*
* Der folgende Code dient als Treiber für ein DS3231-RTC-Gerät, das eine Echtzeituhr betreibt.
* Er wurde im Rahmen eines Projekts im Modul Labor C - Praktikum der Rechnerarchitektur und Systemprogrammierung
* programmiert. Für nähere Informationen über technische Details des Geräts, siehe Moodlekurs.
* Bedeutung der Funktionen und deren Paramter siehe bitte ds3231_drv.h.
*/

#include "ds3231_drv.h"

/* File Operations Datenstruktur für das Modul und für Registrierung der Funktionen */
static struct file_operations ds3231_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = ds3231_read,
        .write          = ds3231_write,
        .open           = ds3231_open,
        .release        = ds3231_close,
};

/*
 * Device-Id. Wird für die Zuordnung des Treibers zum Gerät benötigt.
 * Das für den Treiber passendes Gerät mit der hier definierten ID wird
 * bei der Initialisierung des Moduls hinzugefügt.
 */

static const struct i2c_device_id ds3231_dev_id[] = {
    { "ds3231_drv", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, ds3231_dev_id);

/*
 * I2C Treiber-Struktur. Wird für die Registrierung des Treibers im
 * Linux-Kernel benötigt.
 */

static struct i2c_driver ds3231_driver = {
        .driver = {
                .owner = THIS_MODULE,
                .name  = "ds3231_drv",
        },
        .id_table = ds3231_dev_id,
        .probe	  = ds3231_probe,
        .remove	  = ds3231_remove,
};

static int check_date(ds3231_time_t* time) {
    /*
     * return value = 0 -> Datum ist in Ordnung
     * return value = 1 -> ungütliges Datum eingelesen
     * return value = 2 -> Datum außerhalb des geeigneten Bereiches
     */
    uint8_t retval = 0;

    /* Teste nach ungütligtem Datum */
    if (time->years < 2000 || time->years > 2199) {
        retval = 2;
    } else if (time->seconds < 0 || time->seconds > 59) {
        retval = 1;
    } else if (time->minutes < 0 || time->minutes > 59) {
        retval = 1;
    } else if (time->hours < 0 || time->hours > 23) {
        retval = 1;
    } else if (time->days < 1 || time->days > 31) {
        retval = 1;
    } else if (time->months < 1 || time->months > 12) {
        retval = 1;
    } else {
        /* Falls Monat Februar ist, teste nach Schaltjahren */
        if (time->months == 2) {
            if ((time->years % 4 == 0) && ((time->years % 400 == 0) || (time->years % 100 != 0))) {
                if (time->days > 29) {
                    retval = 1;
                }
            } else if (time->days > 28) {
                retval = 1;
            }
            /* Teste ob bei den Monate 4, 6, 9, 11 höchstens 30 Tage eingegeben wurden */
        } else if (time->months == 4 || time->months == 6 || time->months == 9 || time->months == 11) {
            if (time->days > 30) {
                retval = 1;
            }
        }
    }

    return retval;
}

static int ds3231_open(struct inode *inode, struct file *file) {
    printk("DS3231_drv: ds3231_open aufgerufen\n");
    return 0;
}

static int ds3231_close(struct inode *inode, struct file *file) {
    printk("DS3231_drv: ds3231_close aufgerufen\n");
    return 0;
}

static ssize_t ds3231_read(struct file *file, char __user* puffer, size_t bytes, loff_t *offset) {

    /* Zwischenspeicher für die Uhrzeit */
    uint8_t seconds, minutes, hours, days, months, ret, retval;
    uint16_t years;
    ds3231_time_t time;

    /* Datenstruktur zur Zwischenspeicherung des Status des RTC-Chips */
    ds3231_status_t status;

    /* Bytes, die zu lesen sind */
    ssize_t count;

    /* Wird für die Ausgabe des Datums benutzt */
    char date[30];

    /* Wird für die Benutzung der Monate als Strings verwendet */
    char *list_of_months[12] = {"Januar", "Februar", "März",
                                "April", "Mai", "Juni",
                                "Juli", "August", "September",
                                "Oktober", "November", "Dezember"};

    /* Ausgabe ist schon erfolgt, und keine Bytes übrig. Daher Methode verlassen */
    if (*offset != 0) {
        *offset = 0;
        return 0;
    }

    printk("DS3231_drv: ds3231_read aufgerufen\n");

    /* Reserviere den Datenbus.
     * Methode gibt 1 zurück, falls ein Lock erfolgreich zugewiesen wurde.
     * Methode gibt 0 zurück, falls der Datenbus noch besetzt ist.
     */
    ret = spin_trylock(&the_lock);


    /* Prüfe, ob ret = 0, siehe Bedeutung oben */
    if (!ret) {
        return -EBUSY;
    }

    /* Auslesen des Status des Geräts.
     * Die Daten werden in der Datenstruktur status gespeichert
     */
    status.full    = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_STATUS);
    status.osf     = status.full & DS3231_OSFBIT;
    status.bsy     = status.full & DS3231_BSYBIT;
    status.temp    = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_TEMP);

    /* Prüfe, ob der Oszillator deaktiviert ist.
     * Falls Oszillator deaktiviert ist, aktiviere ihn und gebe eine Fehlermeldung aus,
     * dass nochmal versucht werden soll.
     */
    if ((status.osf >> 7) == 1) {
        status.full = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_CONTROL);
        i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_CONTROL, (status.full | DS3231_BIT_nEOSC));
        i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_STATUS, (status.full & ~DS3231_OSFBIT));
        printk("DS3231: Oszillator ist nicht aktiviert. Wird jetzt aktiviert.\n");
        return -EAGAIN;
        /* Prüfe ob die Temperatur innerhalb des Operating Bereiches ist.
         * Falls nicht, dann gebe eine Warnung aus und setze die Arbeit fort.
         */
    } else if (status.temp > 85 || status.temp < -40) {
        printk("DS3231: ACHTUNG! Temparatur ist nicht im geeigneten Bereich!\n");
    }

    /* Extrahieren der Uhrzeit-Werte aus den jeweiligen Registern und in die temporären Variablen speichern */
    seconds        = i2c_smbus_read_byte_data(ds3231_client, DS3231_SECONDS);
    minutes        = i2c_smbus_read_byte_data(ds3231_client, DS3231_MINUTES);
    hours          = i2c_smbus_read_byte_data(ds3231_client, DS3231_HOURS);
    days           = i2c_smbus_read_byte_data(ds3231_client, DS3231_DATE);
    months         = i2c_smbus_read_byte_data(ds3231_client, DS3231_MONTHS);
    years          = i2c_smbus_read_byte_data(ds3231_client, DS3231_YEARS);

    /* Reservierung des Datenbus aufheben */
    spin_unlock(&the_lock);

    /* In binäre Darstellung bzw. dann später in dezimaler Darstellung  */
    time.seconds = bcd2bin(seconds & DS3231_SECSBITS);
    time.minutes = bcd2bin(minutes & DS3231_MINSBITS);
    time.days    = bcd2bin(days & DS3231_DAYSBITS);
    time.months  = bcd2bin(months & DS3231_MONTHSBITS);
    time.hours   = bcd2bin(hours & DS3231_HRSBITS);
    time.years   = bcd2bin(years & DS3231_YEARSBITS) + 2000;

    /* Prüfe ob das Datum überhaupt gültig ist */
    retval = check_date(&time);

    /* Falls Datum nicht gültig ist, gebe Fehlermeldungen aus.
     * Für Bedeutung der Werte, siehe check_date Methode in ds3231_drv.c
     */
    if (retval == 1) {
        return -ENOEXEC;
    } else if (retval == 2) {
        return -EOVERFLOW;
    }

    /* Speichern der Uhrzeit-Werte in den String date */
    scnprintf(date, sizeof(date), "%02d. %s %02d:%02d:%02d %04d\n",
                time.days, list_of_months[time.months - 1], time.hours,
                time.minutes, time.seconds, time.years);

    /* Menge der auszugebenden Bytes */
    count = strlen(date);

    /* Ausgabe und Rückgabe der nicht gelesenen Bytes */
    bytes = copy_to_user(puffer, date, count);

    /* Wurde bereits gelesen, muss nicht mehr gemacht werden */
    *offset = ~bytes;

    return count;
}


/* Aendert das aktuell gespeicherte Datum */
static ssize_t ds3231_write(struct file *file, const char __user* puffer, size_t bytes, loff_t *offset) {

    /* Zwischenspeicher Datenstruktur für die Uhrzeit */
    ds3231_time_t time;

    /* Datenstruktur zur Zwischenspeicherung des Status des RTC-Chips */
    ds3231_status_t status;

    uint8_t ret, retval;

    /* In diesem Array wird die Eingabe gespeichert */
    char input[30];


    /* Prüfe, ob die Eingabe länger als die Formatlänge wirklich ist
     * Falls ja, dann gebe eine Fehlermeldung aus */
    if (bytes >= 30) {
        return -EOVERFLOW;
    }

    printk("DS3231_drv: ds3231_write aufgerufen\n");

    /* Einlesen des Datums
     * Prüfe, ob keine Bytes verloren gegangen sind
     */
    if (copy_from_user(input, puffer, bytes)) {
        return -EINVAL;
    }

    /* Prüfe, ob versucht wird, die Temperatur manuell zu ändern */
    if (input[0] == '$') {
        sscanf(input + 1, "%hhd %u-%hu-%hu %hu:%hu:%hu", &status.temp, &time.years, &time.months, &time.days, &time.hours, &time.minutes, &time.seconds);
        printk(KERN_ALERT "Temperaturwert: %hhd", status.temp);

        /* Prüfe ob das Datum überhaupt gültig ist */
   	    retval = check_date(&time);

        /* Falls Datum nicht gültig ist, gebe Fehlermeldungen aus.
        * Für Bedeutung der Werte, siehe check_date Methode in ds3231_drv.c
        */
        if (retval == 1) {
            return -ENOEXEC;
        } else if (retval == 2) {
            return -EOVERFLOW;
        }

        /* Konvertierung der Eingabe in BCD-Format */
        time.seconds = bin2bcd(time.seconds);
        time.minutes = bin2bcd(time.minutes);
        time.days    = bin2bcd(time.days);
        time.hours   = bin2bcd(time.hours);
        time.months  = bin2bcd(time.months);
        time.years   = time.years - 2000;
        time.years   = bin2bcd(time.years);

        /* Reserviere den Datenbus
        * Für nähere Informationen der Benutzung der Funktion siehe ds3231_read in ds3231_drv.c
        */
        ret = spin_trylock(&the_lock);

        if (!ret) {
            return -EBUSY;
        }

        /* Auslesen des Status des Geräts.
         * Die Daten werden in der Datenstruktur status gespeichert
         */
        status.full    = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_STATUS);
        status.osf     = status.full & DS3231_OSFBIT;
        status.bsy     = status.full & DS3231_BSYBIT;

    	/* Prüfe, ob der Oszillator deaktiviert ist.
    	 * Falls Oszillator deaktiviert ist, aktiviere ihn und gebe eine Fehlermeldung aus,
    	 * dass nochmal versucht werden soll.
    	 */
    	if ((status.osf >> 7) == 1) {
       		status.full = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_CONTROL);
       		i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_CONTROL, (status.full | DS3231_BIT_nEOSC));
       		i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_STATUS, (status.full & ~DS3231_OSFBIT));
      	 	printk("DS3231: Oszillator ist nicht aktiviert. Wird jetzt aktiviert.\n");
      		return -EAGAIN;
              /* Prüfe ob die Temperatur innerhalb des Operating Bereiches ist.
              * Falls nicht, dann gebe eine Warnung aus und setze die Arbeit fort.
              */
        } else if (status.temp > 85 || status.temp < -40) {
            printk("DS3231: ACHTUNG! Temparatur ist nicht im geeigneten Bereich!\n");
        }

        /* Speichern der Eingabe in das RTC-Chip */
        i2c_smbus_write_byte_data(ds3231_client, DS3231_SECONDS,    time.seconds);
        i2c_smbus_write_byte_data(ds3231_client, DS3231_MINUTES,    time.minutes);
        i2c_smbus_write_byte_data(ds3231_client, DS3231_HOURS,      time.hours);
        i2c_smbus_write_byte_data(ds3231_client, DS3231_DATE,       time.days);
    	i2c_smbus_write_byte_data(ds3231_client, DS3231_MONTHS,     time.months);
    	i2c_smbus_write_byte_data(ds3231_client, DS3231_YEARS,      time.years);

    	/* Aufheben der Reservierung des Datenbus */
    	spin_unlock(&the_lock);
        return bytes;
    }

    /* Eingabe in Array speichern und zuweisen der Uhrzeit und Datum der Datenstruktur */
    sscanf(input, "%u-%hu-%hu %hu:%hu:%hu", &time.years, &time.months, &time.days, &time.hours, &time.minutes, &time.seconds);

    /* Prüfe ob das Datum überhaupt gültig ist */
    retval = check_date(&time);

    /* Falls Datum nicht gültig ist, gebe Fehlermeldungen aus.
    * Für Bedeutung der Werte, siehe check_date Methode in ds3231_drv.c
    */
    if (retval == 1) {
        return -ENOEXEC;
    } else if (retval == 2) {
        return -EOVERFLOW;
    }

    /* Konvertierung der Eingabe in BCD-Format */
    time.seconds = bin2bcd(time.seconds);
    time.minutes = bin2bcd(time.minutes);
    time.days    = bin2bcd(time.days);
    time.hours   = bin2bcd(time.hours);
    time.months  = bin2bcd(time.months);
    time.years   = time.years - 2000;
    time.years   = bin2bcd(time.years);

    /* Reserviere den Datenbus
     * Für nähere Informationen der Benutzung der Funktion siehe ds3231_read in ds3231_drv.c
     */
    ret = spin_trylock(&the_lock);

    if (!ret) {
        return -EBUSY;
    }

    /* Auslesen des Status des Geräts.
     * Die Daten werden in der Datenstruktur status gespeichert
     */
    status.full    = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_STATUS);
    status.osf     = status.full & DS3231_OSFBIT;
    status.bsy     = status.full & DS3231_BSYBIT;
    status.temp    = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_TEMP);

    /* Prüfe, ob der Oszillator deaktiviert ist.
     * Falls Oszillator deaktiviert ist, aktiviere ihn und gebe eine Fehlermeldung aus,
     * dass nochmal versucht werden soll.
     */
    if ((status.osf >> 7) == 1) {
        status.full = i2c_smbus_read_byte_data(ds3231_client, DS3231_REG_CONTROL);
        i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_CONTROL, (status.full | DS3231_BIT_nEOSC));
        i2c_smbus_write_byte_data(ds3231_client, DS3231_REG_STATUS, (status.full & ~DS3231_OSFBIT));
        printk("DS3231: Oszillator ist nicht aktiviert. Wird jetzt aktiviert.\n");
        return -EAGAIN;
    /* Prüfe ob die Temperatur innerhalb des Operating Bereiches ist.
     * Falls nicht, dann gebe eine Warnung aus und setze die Arbeit fort.
     */
    } else if (status.temp > 85 || status.temp < -40) {
        printk("DS3231: ACHTUNG! Temparatur ist nicht im geeigneten Bereich!\n");
    }

    /* Speichern der Eingabe in das RTC-Chip */
    i2c_smbus_write_byte_data(ds3231_client, DS3231_SECONDS,    time.seconds);
    i2c_smbus_write_byte_data(ds3231_client, DS3231_MINUTES,    time.minutes);
    i2c_smbus_write_byte_data(ds3231_client, DS3231_HOURS,      time.hours);
    i2c_smbus_write_byte_data(ds3231_client, DS3231_DATE,       time.days);
    i2c_smbus_write_byte_data(ds3231_client, DS3231_MONTHS,     time.months);
    i2c_smbus_write_byte_data(ds3231_client, DS3231_YEARS,      time.years);

    /* Aufheben der Reservierung des Datenbus */
    spin_unlock(&the_lock);

    return bytes;
}

static int ds3231_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    s32 reg0, reg1;
    u8 reg_cnt, reg_sts;
    int ret;

    printk("DS3231_drv: ds3231_probe aufgerufen\n");

    /* Control und Status Register auslesen.
     */

    reg0 = i2c_smbus_read_byte_data(client, DS3231_REG_CONTROL);
    reg1 = i2c_smbus_read_byte_data(client, DS3231_REG_STATUS);
    if(reg0 < 0 || reg1 < 0) {
        printk("DS3231_drv: Fehler beim Lesen von Control oder Status Register.\n");
	return -ENODEV;
    }
    reg_cnt = (u8)reg0;
    reg_sts = (u8)reg1;
    printk("DS3231_drv: Control: 0x%02X, Status: 0x%02X\n", reg_cnt, reg_sts);

    /* Prüfen ob der Oscilattor abgeschaltet ist, falls ja, muss dieser
     * eingeschltet werden (damit die Zeit läuft).
     */

    if (reg_cnt & DS3231_BIT_nEOSC) {
	printk("DS3231_drv: Oscilator einschalten\n");
	reg_cnt &= ~DS3231_BIT_nEOSC;
    }

    printk("DS3231_drv: Interrupt und Alarms abschalten\n");
    reg_cnt &= ~(DS3231_BIT_INTCN | DS3231_BIT_A2IE | DS3231_BIT_A1IE);

    /* Control-Register setzen */

    i2c_smbus_write_byte_data(client, DS3231_REG_CONTROL, reg_cnt);

    /* Prüfe Oscilator zustand. Falls Fehler vorhanden, wird das Fehlerfalg
     * zurückgesetzt.
     */

    if (reg_sts & DS3231_BIT_OSF) {
	reg_sts &= ~DS3231_BIT_OSF;
	i2c_smbus_write_byte_data(client, DS3231_REG_STATUS, reg_sts);
	printk("DS3231_drv: Oscilator Stop Flag (OSF) zurückgesetzt.\n");
    }

    /* Gerätenummer für das RTC-Gerät reservieren */
    ret = alloc_chrdev_region(&ds3231_device, 0, 1, "ds3231_drv");
    if (ret < 0) {
        printk(KERN_ALERT "DS3231: Fehler bei alloc_chrdev_region()\n");
        return ret;
    }

    /* Schnittstellen und Datenstrukturen initialisieren */
    cdev_init (&cds3231_device, &ds3231_fops);
    ret = cdev_add(&cds3231_device, ds3231_device, 1);
    if (ret < 0) {
        printk(KERN_ALERT "DS3231: Fehler bei registrierung von CDEV-Struktur\n");
        goto unreg_chrdev;
    }

    /* Das Gerät im sysfs registrieren. Erstellen von der Gerät-Datei
     * von dem udev-Dienst
     */
    ds3231_device_class = class_create(THIS_MODULE, "chardev");
    if (ds3231_device_class == NULL) {
        printk(KERN_ALERT "DS3231: Class konnte nicht erstellt werden.\n" );
        goto cleanup_cdev;
    }

    if (device_create(ds3231_device_class, NULL, ds3231_device, NULL, "ds3231_drv") == NULL) {
        printk(KERN_ALERT "DS3231: Device konnte nicht erstellt werden.\n");
        goto cleanup_chrdev_class;
    }

    /* DS3231 erfolgreich initialisiert */

    return 0;

    /* Fehler melden und Ressourcen freigeben */
    cleanup_chrdev_class:
        class_destroy(ds3231_device_class);
    cleanup_cdev:
        cdev_del(&cds3231_device);
    unreg_chrdev:
        unregister_chrdev_region(ds3231_device, 1);
        return -EIO;
}

/* Freigabe der Resourcen.
 * Diese Funktion wird beim Entfernen des Treibers oder Gerätes
 * von Linux-Kernel aufgerufen. Hier sollten die Resourcen, welche
 * in der "ds3231_probe()" Funktion angefordert wurden, freigegeben. */

static int ds3231_remove(struct i2c_client *client) {
    printk("DS3231_drv: ds3231_remove aufgerufen\n");
    device_destroy(ds3231_device_class, ds3231_device);
    class_destroy(ds3231_device_class);
    cdev_del(&cds3231_device);
    unregister_chrdev_region(ds3231_device, 1);
    return 0;
}

 /* Initialisierungsroutine des Kernel-Modules.
 * Wird beim Laden des Moduls aufgerufen. Innerhalb der Funktion
 * wird das neue Device (DS3231) registriert und der I2C Treiber
 * hinzugefügt.
 */

static int __init ds3231_module_init(void) {
    int ret;
    struct i2c_adapter *adapter;
    /* Normaleweise werden die Informationen bezüglich der verbauten
     * Geräten (Devices) während der Kernel-Initialisierung mittels
     * eines Device-Trees Eintrages definiert. Anhand dieser Informationen
     * sucht der Kernel einen passenden Treiber für das Gerät (i2c_device_id).
     * In unserem Fall müssen die Informationen nachträglich definiert
     * und hinzugefügt werden.
     */
    const struct i2c_board_info info = {
            I2C_BOARD_INFO("ds3231_drv", 0x68)
    };

    printk("DS3231_drv: ds3231_module_init aufgerufen\n");
    spin_lock_init(&the_lock);
    ds3231_client = NULL;
    adapter = i2c_get_adapter(1);
    if(adapter == NULL) {
        printk("DS3231_drv: I2C Adapter nicht gefunden\n");
        return -ENODEV;
    }

    /* Neues I2C Device registrieren */
    ds3231_client = i2c_new_client_device(adapter, &info);
    if(ds3231_client == NULL) {
	printk("DS3231_drv: I2C Client: Registrierung fehlgeschlagen\n");
	return -ENODEV;
    }

    /* Treiber registrieren */
    ret = i2c_add_driver(&ds3231_driver);
    if(ret < 0) {
	printk("DS3231_drv: Treiber konnte nicht hinzugefügt werden (errorn = %d)\n", ret);
	i2c_unregister_device(ds3231_client);
	ds3231_client = NULL;
    }

    return ret;
}
module_init(ds3231_module_init);

/* Aufräumroutine des Kernel-Modules.
 * Wird beim Entladen des Moduls aufgerufen. Innerhalb der Funktion
 * werden alle Resourcen wieder freigegeben.
 */
static void __exit ds3231_module_exit(void) {
	printk("DS3231_drv: ds3231_module_exit aufgerufen\n");
	if(ds3231_client != NULL) {
		i2c_del_driver(&ds3231_driver);
		i2c_unregister_device(ds3231_client);
		ds3231_client = NULL;
	}
}
module_exit(ds3231_module_exit);

/* Module-Informationen. */
MODULE_AUTHOR("Bashar Khoulani, uk083781@~, Adnan Noah Nezovic uk083333@~");
MODULE_DESCRIPTION("RTC DS3231 Treiber, verwaltet durch ein Raspberry Pi. Projekt fuer SysProg2022");
MODULE_LICENSE("GPL");
