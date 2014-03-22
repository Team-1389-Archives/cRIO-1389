#ifndef IMAGE_ANALYSIS_CLIENT_H
#define IMAGE_ANALYSIS_CLIENT_H
#include <pthread.h>
#ifndef COMPUTER_TEST
#include <WPILib.h>
#endif

#define IMAGE_ANALYSIS_SERVER_IP		("10.13.89.10")
//See the FIRST FMS White Paper to find out what ports are available http://bitly.com/1abqwix
#define IMAGE_ANALYSIS_SERVER_PORT		(8080)

struct ImageData {
    float x;
    float y;
    float radius;
};

class ImageAnalysisClient {
public:
    ImageAnalysisClient(const char *address, int port);
    virtual ~ImageAnalysisClient();

    void copyImageData(ImageData *data);

    void _threadEntry();//This method must be public, though it should not be called
private:
    void setImageData(ImageData *data);
    char *m_address;
    int m_port;
    ImageData m_image_data;
    pthread_t m_thread;
    pthread_mutex_t m_lock;
    #ifndef COMPUTER_TEST
    DriverStationLCD *m_lcd;
    #endif
};

#endif
