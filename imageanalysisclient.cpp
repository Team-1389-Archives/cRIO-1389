#include "imageanalysisclient.h"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sockLib.h>
#include <netinet/in.h>
#include <sched.h>
#include <stdio.h>
#include <vxWorks.h> 
#include <errno.h>
#include <fioLib.h>
#include <hostLib.h> 
#include <inetLib.h> 
#include <signal.h>
#include <sigLib.h>
#include <usrLib.h>

//NOTE: the static_cast<void*> are being put in for safety. In the past i've had problems with using void* as they are used in C

class Locker {
public:
    Locker(pthread_mutex_t *lock): m_lock(lock) {
        pthread_mutex_lock(m_lock);
    }
    virtual ~Locker() {
        pthread_mutex_unlock(m_lock);
    }
private:
    pthread_mutex_t *m_lock;
};

static void* image_analysis_communication_thread(void *cls) {
    while(true) {
        static_cast<ImageAnalysisClient*>(cls)->_threadEntry();
        sched_yield();
    }
    return NULL;
}

ImageAnalysisClient::ImageAnalysisClient(const char *address, int port)
    : m_address(address), m_port(port) {
    memset(static_cast<void*>(&m_image_data), 0, sizeof(m_image_data));
    pthread_mutex_init(&m_lock, NULL);
    pthread_create(&m_thread, NULL, image_analysis_communication_thread, static_cast<void*>(this));
}

ImageAnalysisClient::~ImageAnalysisClient() {
    pthread_kill(m_thread, 0);//TODO: is there a better way to cleanup?
    pthread_mutex_destroy(&m_lock);
}

void ImageAnalysisClient::copyImageData(ImageData *id) {
    Locker locker(&m_lock);
    memcpy(
        static_cast<void*>(id),
        static_cast<void*>(&m_image_data),
        sizeof(ImageData)
    );
}

void ImageAnalysisClient::setImageData(ImageData *data) {
    Locker locker(&m_lock);
    memcpy(
        static_cast<void*>(&m_image_data),
        static_cast<void*>(data),
        sizeof(ImageData)
    );
}

#define REQUEST		("ask\n")

void ImageAnalysisClient::_threadEntry() {
    char buffer[1024];
    int buffer_len=0;
    int sock=socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in dst;
    memset(static_cast<void*>(&dst), 0, sizeof(dst));
    dst.sin_family=AF_INET;
    dst.sin_addr.s_addr=inet_addr(m_address);
    dst.sin_port=htons(m_port);

    if(connect(sock, (struct sockaddr*)&dst, sizeof(struct sockaddr))==-1) {
        return;
    }
    while(true) {
        //This is the main loop
        if(write(sock, REQUEST, sizeof(REQUEST))!=sizeof(REQUEST)) {
            //While this could occur if the connection is still established, but the kernel buffer is just filled, that is an unlikely scenario.
            close(sock);
            return;
        }
        while(true) {
            //This is the read loop
            if(buffer_len==sizeof(buffer)) {
                close(sock);
                return;
            }
            int len=read(sock, buffer+buffer_len, sizeof(buffer)-buffer_len);
            if(len<=0) {
                close(sock);
                return;
            }
            buffer_len+=len;
            int i;
            //Linear search backward for linefeed. It's more likely that it's at the end of the buffer
            for(i=buffer_len-1; i>=0 && buffer[i]!='\n'; i--) {}
            if(buffer_len>0) {
                //If it equals zero, then it's the first character
                ImageData data;
                buffer[i]='\0';//Replace the linefeed with the NULL terminator so that scanf will work
                sscanf(buffer, "%f %f %f", &data.x, &data.y, &data.radius);
                setImageData(&data);
                if(i==buffer_len-1) {
                    //The linefeed was the last character read, so just clear out the buffer
                    buffer_len=0;
                } else {
                    //There is more stuff remaining in the buffer. Let's just move it to the beginning, so that we don't need a cyclic buffer.
                    memmove(
                        static_cast<void*>(buffer),
                        static_cast<void*>(buffer+i),
                        buffer_len-i
                    );
                    buffer_len=buffer_len-i;
                }
            }
        }
    }
}
