#ifndef VLR_CAN_H
#define VLR_CAN_H

namespace dgc {

class Canbus {
  public:
    Canbus();
    ~Canbus();

    void Open();
    void Close(); 
    int ReadMessage();
    int SendMessage();
};

class CanInterface {
  public:
    //virtual ~CanInterface() {};
    virtual bool Open();
    virtual void Close();
    virtual int ReadMessage(long *id, void *data);
    virtual int SendMessage(long id, void *data, int len);
};

}

#endif //VLR_CAN_H
