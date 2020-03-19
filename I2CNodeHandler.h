#ifndef I2C_NODE_HANDLER_H_
#define I2C_NODE_HANDLER_H_


#include <Arduino.h>
#include <Wire.h>

#define BUFFER_SIZE 32
#define CB_MAX_NUM  20

class I2CSlaveNode
{
public:
    typedef enum FrameMode {
        Standby   = 0 ,
        Receive       ,
        Request       ,
        Response      ,
    } frame_mode_t;

    virtual void _res_cb(uint8_t*, int &) = 0;
    virtual void _req_cb(uint8_t*, int &) = 0;

    I2CSlaveNode(uint8_t sub_addr)
      : _sub_addr(sub_addr)
    {
        _mode = Standby;
    }

    frame_mode_t get_mode() const 
    {
        return _mode;
    }

    static void _onReceiveCb(const int data_size, I2CSlaveNode *node)
    {  
        frame_mode_t mode = (frame_mode_t)Wire.read();
        int _data_size = data_size - 1;
        
        if (_data_size >= 0 && _data_size < BUFFER_SIZE) {
            int i = 0;
            for (; i < _data_size; i++) {
                node->_data[i] = Wire.read();
            }

            switch(mode) {
                case Receive :
                    if(node->_mode != Response) {
                        node->_mode = mode;
                        node->_res_cb(node->_data, _data_size);
                    }
                break;
                case Request :
                    node->_mode = Response;
                break;
                default :
                //nop
                break;
            };
        }
    }

    static int _onRequestCb(uint8_t* data, I2CSlaveNode *node)
    {
        node->_mode = Standby;
        int data_size = 0;
        node->_req_cb(data, data_size);
        if(data_size <= BUFFER_SIZE - 1)
            return data_size;
        else
            return BUFFER_SIZE - 1;  
    }

    uint8_t _sub_addr;
    frame_mode_t _mode;
    uint8_t _data[BUFFER_SIZE];
};

class I2CNodeHandler
{
private:
    static I2CSlaveNode *_cb[CB_MAX_NUM];

public:
    I2CNodeHandler(){}

    void begin(uint8_t my_addr, int bus_frequency = 100000)
    {
        Wire.begin(my_addr);
        Wire.setClock(bus_frequency);
        Wire.onReceive(_onReceive_task);
        Wire.onRequest(_onRequest_task);
    }

    int add_node(I2CSlaveNode *cb)
    {
        int i = 0;
        for(; i <= CB_MAX_NUM; i++){
            if(_cb[i] == NULL){
                _cb[i] = cb;
                break;
            }
        }
        return i;
    }

    int get_node_num()
    {
        int i = 0;
        for(; i <= CB_MAX_NUM; i++){
            if(_cb[i] == NULL){
                break;
            }
        }
        return i;
    }
    
private:
    static void _onReceive_task(int frame_size)
    {   
        int sub_addr = 0;
        if (frame_size > 0 && frame_size < BUFFER_SIZE) {
            sub_addr = Wire.read();
            for(int i = 0; _cb[i] != NULL; i++) {
                if (_cb[i]->_sub_addr == sub_addr) {
                    _cb[i]->_onReceiveCb(frame_size - 1, _cb[i]);
                    break;
                }
            }
        }
    }

    static void _onRequest_task()
    {
        uint8_t data[BUFFER_SIZE] = {};
        for(int i = 0; _cb[i] != NULL; i++) {
            if (_cb[i]->_mode == I2CSlaveNode::Response) {
                int frame_size = 2;
                data[0] = _cb[i]->_sub_addr;
                data[1] = (uint8_t)_cb[i]->_mode;
                frame_size += _cb[i]->_onRequestCb(data + 2, _cb[i]);
                Wire.write(data, frame_size);
                break;
            }
        }
    }
};

I2CSlaveNode *I2CNodeHandler::_cb[CB_MAX_NUM];

#endif