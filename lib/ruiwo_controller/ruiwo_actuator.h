#ifndef RUIWO_ACTUATOR_H
#define RUIWO_ACTUATOR_H

#include <Python.h>
#include <vector>
#include <thread>
#include <iostream>
static PyObject *RuiWo_pJoinMethod;// 用于将python线程移动到c++线程中,避免GIL占用

class RuiWoActuator
{
private:
    PyObject *pModule;
    PyObject *RuiWoActuatorClass;
    PyObject *ActuatorInstance;

    PyObject *pEnableMethod;
    PyObject *pCloseMethod;
    PyObject *pDisableMethod;
    PyObject *pSetPositionMethod;
    PyObject *pGetPositionMethod;
    PyObject *pGetJointStateMethod;
    std::string pymodule_path;
    PyGILState_STATE gstate;
    std::thread pythonThread;

public:
    // 需要传入python模块所在路径
    RuiWoActuator(std::string pymodule_path = "");

    ~RuiWoActuator();

    int initialize();
    void enable();
    void disable();
    void close();
    void join();
    // void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions ,std::vector<double> vel ,std::vector<double> pos_kp ,std::vector<double> pos_kd,std::vector<double> torque);
    void set_positions(const std::vector<uint8_t> &ids, const std::vector<double> &positions);
    std::vector<double> get_positions();
    std::vector<std::vector<double>> get_joint_state();
};

#endif // RUIWO_ACTUATOR_H
