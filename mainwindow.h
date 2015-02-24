#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "rtl-sdr.h"
#include <QMainWindow>
#include <QMessageBox>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>
#include <QtCore>
#include <QTreeWidget>
#include <QMainWindow>
#include <QWidget>
#include <QDebug>
#include <QTimer>
#include <QMouseEvent>
#include <QVector>
#include <QTcpSocket>



#define MODES_AUTO_GAIN            -100         /* Use automatic gain. */
#define MODES_MAX_GAIN             999999       /* Use max available gain. */
#define MODES_DEFAULT_RATE         2000000
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_INTERACTIVE_REFRESH_TIME 250      /* Milliseconds */
#define MODES_INTERACTIVE_ROWS 15               /* Rows on screen */
#define MODES_INTERACTIVE_TTL 10 //60 deafult                /* TTL before being removed */

#define MODES_ICAO_CACHE_LEN 1024 /* Power of two required. */
#define MODES_ICAO_CACHE_TTL 60   /* Time to live of cached addresses. */
#define MODES_UNIT_FEET 0
#define MODES_UNIT_METERS 1

#define MODES_ASYNC_BUF_NUMBER     12
#define MODES_DATA_LEN             (16*16384)   /* 256k */

#define MODES_PREAMBLE_US 8       /* microseconds */
#define MODES_LONG_MSG_BITS 112
#define MODES_SHORT_MSG_BITS 56
#define MODES_FULL_LEN (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
#define MODES_LONG_MSG_BYTES (112/8)
#define MODES_SHORT_MSG_BYTES (56/8)
#define MODES_NOTUSED(V) ((void) V)


#define MODES_DEBUG_DEMOD (1<<0)
#define MODES_DEBUG_DEMODERR (1<<1)
#define MODES_DEBUG_BADCRC (1<<2)
#define MODES_DEBUG_GOODCRC (1<<3)
#define MODES_DEBUG_NOPREAMBLE (1<<4)
#define MODES_DEBUG_NET (1<<5)
#define MODES_DEBUG_JS (1<<6)

/* When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
 * at least greater than a given level for us to dump the signal. */
#define MODES_DEBUG_NOPREAMBLE_LEVEL 25
struct {
    /* Internal state */
    pthread_t reader_thread;
    pthread_t proccess_thread;
    pthread_mutex_t data_mutex;     /* Mutex to synchronize buffer access. */
    pthread_cond_t data_cond;       /* Conditional variable associated. */
    unsigned char *data;            /* Raw IQ samples buffer */
    uint16_t *magnitude;            /* Magnitude vector */
    uint32_t data_len;              /* Buffer length. */
    int fd;                         /* --ifile option file descriptor. */
    int data_ready;                 /* Data ready to be processed. */
    uint32_t *icao_cache;           /* Recently seen ICAO addresses cache. */
    uint16_t *maglut;               /* I/Q -> Magnitude lookup table. */
    int exit;                       /* Exit from the main loop when true. */

    /* RTLSDR */
    int dev_index;
    int gain;
    int enable_agc;
    rtlsdr_dev_t *dev;
    int freq;

    /* Configuration */
    char *filename;                 /* Input form file, --ifile option. */
    int fix_errors;                 /* Single bit error correction if true. */
    int check_crc;                  /* Only display messages with good CRC. */
    int raw;                        /* Raw output format. */
    int debug;                      /* Debugging mode. */
    int net;                        /* Enable networking. */
    int net_only;                   /* Enable just networking. */
    int interactive;                /* Interactive mode */
    int interactive_rows;           /* Interactive mode: max number of rows. */
    int interactive_ttl;            /* Interactive mode: TTL before deletion. */
    int stats;                      /* Print stats at exit in --ifile mode. */
    int onlyaddr;                   /* Print only ICAO addresses. */
    int metric;                     /* Use metric units. */
    int aggressive;                 /* Aggressive detection algorithm. */

    /* Interactive mode */
    struct aircraft *aircrafts;
    long long interactive_last_update;  /* Last screen update in milliseconds */

    /* Statistics */
    long long stat_valid_preamble;
    long long stat_demodulated;
    long long stat_goodcrc;
    long long stat_badcrc;
    long long stat_fixed;
    long long stat_single_bit_fix;
    long long stat_two_bits_fix;
    long long stat_http_requests;
    long long stat_sbs_connections;
    long long stat_out_of_phase;
} Modes;
/* Structure used to describe an aircraft in iteractive mode. */
struct aircraft {
    uint32_t addr;      /* ICAO address */
    char hexaddr[7];    /* Printable ICAO address */
    char flight[9];     /* Flight number */
    int altitude;       /* Altitude */
    int speed;          /* Velocity computed from EW and NS components. */
    int track;          /* Angle of flight. */
    time_t seen;        /* Time at which the last packet was received. */
    long messages;      /* Number of Mode S messages received. */
    /* Encoded latitude and longitude as extracted by odd and even
     * CPR encoded messages. */
    int odd_cprlat;
    int odd_cprlon;
    int even_cprlat;
    int even_cprlon;
    double lat, lon;    /* Coordinated obtained from CPR encoded data. */
    long long odd_cprtime, even_cprtime;
    struct aircraft *next; /* Next aircraft in our linked list. */
};
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    void *readTCPdata(void);
    void *mainloopC(void);

    //static void* readerThreadEntryPoint(void *arg) { return ((MainWindow*)arg)->readTCPdata(); }//in C++ i think work only as static
    static void *readerThreadEntryPoint(void *arg);
    static void *procThreadEntryPoint(void *arg) { return ((MainWindow*)arg)->mainloopC(); }//in C++ i think work only as static
    explicit MainWindow(QWidget *parent = 0);    ~MainWindow();

private slots:

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;
    void modesInitRTLSDR(void);
    void modesInitConfig(void);
    void modesInit(void);
    void backgroundTasks(void);
    static long long mstime(void);
    void interactiveRemoveStaleAircrafts(void);
    void interactiveShowData(void);
    //**void mainloopC(void);
   // static void *readerThreadEntryPoint(void *arg); //in C++ i think work only as static
    static void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx); //in C++ i think work only as static
    void computeMagnitudeVector(void);
    void detectModeS(uint16_t *m, uint32_t mlen);
    void dumpRawMessage(char *descr, unsigned char *msg,uint16_t *m, uint32_t offset);
    int detectOutOfPhase(uint16_t *m);
    void applyPhaseCorrection(uint16_t *m);

    int modesMessageLenByType(int type);
    void decodeModesMessage(struct modesMessage *mm, unsigned char *msg);
    void useModesMessage(struct modesMessage *mm);
    struct aircraft *interactiveReceiveData(struct modesMessage *mm);
    struct aircraft *interactiveFindAircraft(uint32_t addr);
    struct aircraft *interactiveCreateAircraft(uint32_t addr);

    uint32_t modesChecksum(unsigned char *msg, int bits);
    int fixSingleBitErrors(unsigned char *msg, int bits);
    int fixTwoBitsErrors(unsigned char *msg, int bits);
    int bruteForceAP(unsigned char *msg, struct modesMessage *mm);
    void addRecentlySeenICAOAddr(uint32_t addr);
    uint32_t ICAOCacheHashAddress(uint32_t a);
    int ICAOAddressWasRecentlySeen(uint32_t addr);

    int decodeAC12Field(unsigned char *msg, int *unit);
    int decodeAC13Field(unsigned char *msg, int *unit);
    void decodeCPR(struct aircraft *a);


    double cprDlonFunction(double lat, int isodd);
    int cprNFunction(double lat, int isodd);
    int cprNLFunction(double lat);
    int cprModFunction(int a, int b);
//GUIs functions
    void addTreeRoot(QStringList B);
    void closeEvent ( QCloseEvent * event );
    QVector<aircraft> AirplanesVector;
    int ReturnIndexOfSearch(QString Name);\
};

#endif // MAINWINDOW_H
