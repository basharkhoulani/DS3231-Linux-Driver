/* @file ds3231_drv.h
 * @brief Header-Datei für das Treiber-Programm ds3231_drv.c.
 *
 * Diese Datei enthält das Importieren wichtiger Linux-Bibliotheken,
 * definiert essentielle Addressen und Definitonen der Datum und Uhrzeit-Registern,
 * erstellt Prototype der Funktionen und deklariert wichtige Variablen.
 *
 * @author Bashar Khoulani
 * @author Adnan Noah Nezovic
 */

#include <linux/slab.h>
#include <linux/bcd.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <asm/errno.h>
#include <asm/delay.h>

/* Register Misc. Definitionen */
#define DS3231_REG_CONTROL	0x0e
#define DS3231_BIT_nEOSC	0x80
#define DS3231_BIT_INTCN	0x04
#define DS3231_BIT_A2IE		0x02
#define DS3231_BIT_A1IE		0x01
#define DS3231_REG_STATUS	0x0f
#define DS3231_BIT_OSF		0x80
#define DS3231_REG_TEMP     0x11

/* Datum und Uhrzeit Register Definitionen */
#define DS3231_SECONDS		0x00
#define DS3231_MINUTES		0x01
#define DS3231_HOURS		0x02
#define DS3231_DAYS		    0x03
#define DS3231_DATE		    0x04
#define DS3231_MONTHS		0x05
#define DS3231_YEARS		0x06

/* Definitonen für das Auslesen der Uhrzeit */
#define DS3231_SECSBITS     0b01111111

#define DS3231_MINSBITS     0b01111111

#define DS3231_HRSBITS      0b00111111
#define DS3231_12n24        0b01000000

#define DS3231_DAYSBITS     0b00111111

#define DS3231_MONTHSBITS   0b00011111
#define DS3231_CENTURYBITS  0b10000000

#define DS3231_YEARSBITS    0b11111111

/* Definitonen für das Auslesen des Status des RTC-Chips */
#define DS3231_OSFBIT       0b10000000
#define DS3231_BSYBIT       0b00000100

/* Datenstruktur zur Speicherung und Übergeben der Uhrzeit */
typedef struct time {
    uint16_t seconds, minutes, hours, months, days;
    uint32_t years;
} ds3231_time_t;

/* Datenstruktur zur Speicherung und Übergeben des Status des RTC-Chips */
typedef struct status {
    uint8_t osf, bsy, full;
    int8_t temp;
} ds3231_status_t;

/* Prototype und Dokumentation der Funktionen */

/* @brief Initialisierung des Treibers und Devices.
 *
 * Diese Funktion wird von Linux-Kernel aufgerufen, aber erst nachdem ein zum
 * Treiber passende Device-Information gefunden wurde. Innerhalb der Funktion
 * wird der Treiber und das Device initialisiert.
 *
 * @param *client I2C-RTC-Client
 * @param *id ID des Geräts
 * @return int 0, wenn erfolgreich. 1 (oder größer), sonst.
 */
static int ds3231_probe(struct i2c_client *client, const struct i2c_device_id *id);

/* @brief Schließt den Treiber
 *
 * Irrelevant für die Verwendung des Treibers
 *
 * @param *inode Enthält den „inode“-Eintrag der zugehörigen Gerätedatei
 * @param *file Enthält Informationen über den Aufrufer und die Art des Aufrufs
 * @return int 0, wenn erfolgreich. 1 (oder größer), sonst.
 */
static int ds3231_open(struct inode *inode, struct file *file);

/* @brief Schließt den Treiber
 *
 * Irrelevant für die Verwendung des Treibers
 *
 * @param *inode Enthält den „inode“-Eintrag der zugehörigen Gerätedatei
 * @param *file Enthält Informationen über den Aufrufer und die Art des Aufrufs
 * @return int 0, wenn erfolgreich. 1 (oder größer), sonst.
 */
static int ds3231_close(struct inode *inode, struct file *file);

/* @brief Liest die Uhrzeit aus und gibt diese dem Benutzer aus
 *
 * Die Funktion kann ein Datum aus und dieses in den Userspace ausgeben. Es muss aber ein Format beachtet werden,
 * nämlich DD. M hh:mm:ss YYYY. Beispiel: 15. April 19:34:56 2019. Es wird außerdem geprüft, ob das gespeicherte Datum
 * außerhalb des Operating Bereiches ist.
 *
 * @param *file Enthält Informationen über den Aufrufer und die Art des Aufrufs
 * @param *puffer Speicherbereich im Userspace, in den die zu lesenden Daten kopiert werden
 * @param bytes  Anzahl der zu lesenden Bytes
 * @param *offset Offset innerhalb der Datei bzw. des Gerätes
 * @return ssize_t Anzahl der gelesenen Bytes
 */
static ssize_t ds3231_read(struct file *file, char __user* puffer, size_t bytes, loff_t *offset);

/* @brief Liest eine Eingabe von dem Benutzer ein und speichert diese Eingabe in die jeweilige Datenstruktur
 *
 * Die Funktion kann ein Datum einlesen und dieses in das RTC-Chip speichern. Es muss aber ein Format beachtet werden,
 * nämlich YYYY-MM-DD hh:mm:ss. Beispiel: 2019-04-15 19:34:56. Bei ungültigen Formaten wird -EINVAL, invalid Argument,
 * ausgegeben.
 * Außerdem kann die Funktion den Temperaturwert manuell ändern, indem man bei der Eingabe als erstes Zeichen $ und danach
 * den gewünschten Wert eingeben.
 *
 * @param *file Enthält Informationen über den Aufrufer und die Art des Aufrufs
 * @param *puffer Speicherbereich im Userspace, in den die zu lesenden Daten kopiert werden
 * @param bytes  Anzahl der zu lesenden Bytes
 * @param *offset Offset innerhalb der Datei bzw. des Gerätes
 * @return ssize_t Anzahl der gelesenen Bytes
 */
static ssize_t ds3231_write(struct file *file, const char __user* puffer, size_t bytes, loff_t *offset);

/* @brief Entfernt den I2C-Client des RTC-Chips
 *
 * Löscht und entfernt alle Registrierungen gemacht von dem Drivercode ds3231_drv.c
 *
 * @param *client I2C-RTC-Client
 * @return 0, falls erfolgreich. 1 (oder größer), sonst.
 */
static int ds3231_remove(struct i2c_client *client);

/* @brief Prüft die Datenstruktur, ob das gespeicherte Datum gültig ist
 *
 * Wenn die gespeicherte Uhrzeit und das Datum ungültig sind, dann wird die Fehlermeldung
 * -ENOEXEC, No Execution, ausgegeben. Dies passiert bspw., wenn Minute 75 oder Stunde 25 eingegeben wurde.
 * Die Funktion prüft des Weiteren, ob das gespeicherte Jahr ein Schaltjahr ist und falls ja, erweitert den
 * Prüfungsbereich der Tage.
 * Die Funktion prüft außerdem nach dem gespeicherten und einzulesenden Jahr, ob das außerhalb des Operating Bereiches ist.
 * Falls ja, dann wird eine Fehlermeldung -EOVERFLOW, Overflow, ausgegeben.
 *
 * @param *time Datenstruktur für die Uhrzeit und Datum
 * @return 0, falls gültig. 1, falls ungültig. 2, falls außerhalb Operating Bereiches.
 */
static int check_date(ds3231_time_t* time);

/* Deklarationen für Registrierung der Geräte-Dateien */
static dev_t ds3231_device;
static struct cdev cds3231_device;
static struct class *ds3231_device_class;

/*
 * Der Zeiger wird bei Initialisierung gesetzt und wird für die
 * i2c Kommunikation mit dem  Device (DS3231) benötigt.
 */
static struct i2c_client *ds3231_client;

/* Globale Variable zur Benutzung einer Synchronisationsmethode für den gleichzeitigen Zugriff auf das RTC-Chip */
static spinlock_t the_lock;
