#include "imageanalysisclient.h"
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#ifndef COMPUTER_TEST
#include <sockLib.h>
#include <vxWorks.h> 
#include <errno.h>
#include <fioLib.h>
#include <hostLib.h> 
#include <inetLib.h> 
#include <signal.h>
#include <sigLib.h>
#include <usrLib.h>
#include "WPILib.h"
#else
#include <iostream>
using namespace std;
#endif

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
#ifndef COMPUTER_TEST
	DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line1, "IA: About to connect");
	DriverStationLCD::GetInstance()->UpdateLCD();
#endif
	while(true) {
        static_cast<ImageAnalysisClient*>(cls)->_threadEntry();
        sched_yield();
    }
    return NULL;
}

ImageAnalysisClient::ImageAnalysisClient(const char *address, int port)
    : m_port(port) {
    #ifndef COMPUTER_TEST
	m_lcd = DriverStationLCD::GetInstance();
	#endif
    m_address = strdup(address);
    memset(static_cast<void*>(&m_image_data), 0, sizeof(m_image_data));
    pthread_mutex_init(&m_lock, NULL);
    pthread_create(&m_thread, NULL, image_analysis_communication_thread, static_cast<void*>(this));
}

ImageAnalysisClient::~ImageAnalysisClient() {
    //pthread_kill(m_thread, 0);//TODO: is there a better way to cleanup?
    pthread_mutex_destroy(&m_lock);
    free(m_address);
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
	//VxWorks strerror_r has a buffer overflow exploit
#ifndef COMPUTER_TEST
#define LCD_ERRNO(name)		do{strerror_r(errno, buffer/*, sizeof(buffer)-1*/);	\
							m_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "IA: %s(): %s", name, buffer);	\
							m_lcd->UpdateLCD();}while(0)
#define LCD_MSG(msg)		do{m_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "IA: %s", msg);	\
							m_lcd->UpdateLCD();}while(0)
#else
#define LCD_ERRNO(x)        do{perror(x);abort();}while(0)
#define LCD_MSG(x)          (cerr<<"MSG: "<<(x)<<endl)
#endif
	
	
    FILE *f;
    int sock=socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in dst;
    memset(static_cast<void*>(&dst), 0, sizeof(dst));
    dst.sin_family=AF_INET;
    dst.sin_addr.s_addr=inet_addr(m_address);
    dst.sin_port=htons(m_port);

    if(connect(sock, (struct sockaddr*)&dst, sizeof(struct sockaddr))==-1) {
    	LCD_ERRNO("connect()");
        return;
    }
    f=fdopen(sock, "r+");
    while(true) {
        //This is the main loop
        if(fprintf(f, "%s", REQUEST)<0) {
        	LCD_ERRNO("fprintf()");
        	//While this might occur if the connection is still established, but the kernel buffer is just filled, that is an unlikely scenario.
            fclose(f);
            return;
        }
        fflush(f);
        ImageData data;
        float parts[3];//Because of padding, it's safer to not just assume that the struct is in the same format as this array.
        for(int part_idx=0;part_idx<(sizeof(parts)/sizeof(float));part_idx++){
            char number_buffer[64];
            int buffer_pos=0;
            while(true){
                char ch=fgetc(f);
                if(ch==EOF){
                    LCD_ERRNO("fgetc()");
                    fclose(f);
                    return;
                }else if(isspace(ch)){
                    number_buffer[buffer_pos]='\0';
                    break;
                }else{
                    number_buffer[buffer_pos]=ch;
                }
                buffer_pos++;
            }
            parts[part_idx]=atof(number_buffer);
        }
        data.x=parts[0];
        data.y=parts[1];
        data.radius=parts[2];
        setImageData(&data);
    }
#undef LCD_ERRNO
#undef LCD_MSG
}
