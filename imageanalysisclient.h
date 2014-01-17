#ifndef IMAGE_ANALYSIS_CLIENT_H
#define IMAGE_ANALYSIS_CLIENT_H
#include <pthread.h>

#define IMAGE_ANALYSIS_SERVER_IP		("10.13.89.6")
//See the FIRST FMS White Paper to find out what ports are available http://bitly.com/1abqwix
#define IMAGE_ANALYSIS_SERVER_PORT		(443)

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

    void _threadEntry();//This method must be public, thought it should not be called
private:
    void setImageData(ImageData *data);
    const char *m_address;
    int m_port;
    ImageData m_image_data;
    pthread_t m_thread;
    pthread_mutex_t m_lock;
};

#endif
