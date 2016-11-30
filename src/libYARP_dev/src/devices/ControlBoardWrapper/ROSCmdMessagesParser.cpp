/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <stdexcept>      // std::out_of_range

#include "ControlBoardWrapper.h"
#include "ROSCmdMessagesParser.h"
#include <iostream>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::impl;
using namespace yarp::sig;
using namespace std;


ROSCmdMessagesParser::ROSCmdMessagesParser() : joint_num(0) {}

bool ROSCmdMessagesParser::inputMsgIsValid()
{
    // TBD: this function should either be mutexed or have the input msg
    // as a parameter.

    // collect here all checks for a valid input message
    size_t cmdLenght = inputMsg.names.size();
    if( cmdLenght <= 0)
    {
        yError() << "Input message has mandatory field 'names' missing";
        return false;
    }

    if( inputMsg.reference.size() <= 0)
    {
        yError() << "Input message has mandatory field 'reference' missing. Name size is " << cmdLenght << " while cmd lenght is " << inputMsg.reference.size();
        return false;
    }

    if( cmdLenght != inputMsg.reference.size())
    {
        yError() << "Input message has fields 'names' and 'reference' of different sizes";
        return false;
    }
    return true;
}

bool ROSCmdMessagesParser::read(yarp::os::ConnectionReader& connection)
{
    inputMsg.read(connection);

    if(!inputMsgIsValid())
    {
        yTrace() << "\n\tCallback read failed because input ROS msg was not valid";
        return false;
    }

    int cmdLenght = inputMsg.names.size();

    // process data in inputMsg
    printf("-------------------------------\nGot\n");
    printf("Size of name %d\n", cmdLenght);

    int _j;
    for(int i=0; i<cmdLenght; i++)
    {
        try
        {
            _j = jointMap.at(inputMsg.names[i]);
        }
        catch (const std::out_of_range & oor)
        {
            yError() << "Non existing joint " << inputMsg.names[i] << " found at index " << i << " of the input ROS msg";
            continue;
        }

        printf("Mode %d\n", inputMsg.referenceType[i]);
        printf("Name %s  --[%d]\n", inputMsg.names[i].c_str(), _j);
        printf("Comm %f\n", inputMsg.reference[_j]);
        stream_IPosCtrl2->positionMove(_j, inputMsg.reference[i] * 180/3.14);
    }

    return true;
}


bool ROSCmdMessagesParser::initialize(ControlBoardWrapper *x)
{
    stream_IPosCtrl2    = dynamic_cast<yarp::dev::IPositionControl2 *> (x);
    stream_IPosDirect   = dynamic_cast<yarp::dev::IPositionDirect *> (x);
    stream_IVel2        = dynamic_cast<yarp::dev::IVelocityControl2 *> (x);
    stream_IOpenLoop    = dynamic_cast<yarp::dev::IOpenLoopControl *> (x);
    stream_ITorque      = dynamic_cast<yarp::dev::ITorqueControl *> (x);

    if( ! (stream_IPosCtrl2 && stream_IVel2 && stream_IPosDirect && stream_IOpenLoop && stream_ITorque))
    {
      yError() << "Internal error while opening ControlBoardWrapper device. \n\t"\
                   "Adv: ROSCmdMessagesParser initialize(...) failed to get the required interfaces.";
      return false;
    }
    // so far so good
    if(! stream_IPosCtrl2->getAxes(&joint_num))
    {
        if(joint_num <= 0)
            yError() << "Internal error while opening ControlBoardWrapper device. \n\t" \
                        "Adv: ROSCmdMessagesParser initialize(...) failed to get the number of joints.";
        return false;
    }

    std::pair<std::map<std::string, int>::iterator,bool> ret;

    int joint_num;
    x->getAxes(&joint_num);
    ConstString name;

    yError() << "joint num :" << joint_num;
    for(int _j=0; _j<joint_num; _j++)
    {
        x->getAxisName(_j, name);
        ret = jointMap.insert(std::pair<std::string, int>(name, _j) );
        if (ret.second == false)
        {
            yError() << "element " << name <<  " already existed";
        }
        else
            yError() << "added element " << name <<  " at index " << _j;
    }
    return true;
}

#if 0
// streaming port callback
void ROSCmdMessagesParser::onRead(robotology_msgs_controlBoardMsg& rosCmd)
{

yTrace() << "\n\t" << rosCmd.getTypeText() << " rosCmd.reference " << rosCmd.reference;
/*
 *    //Use the following only for debug, since it can heavily slow down the system
 *    //yDebug("Received reference %s, %s\n", b.toString().c_str(), cmdVector.toString().c_str());
 *
 *    // some consistency checks
 *    if ((int)cmdVector.size() > stream_nJoints)
 *    {
 *        yarp::os::ConstString str = yarp::os::Vocab::decode(b.get(0).asVocab());
 *        yError("Received reference vector with number of elements bigger than axis controlled by this wrapper (cmd: %s requested jnts: %d received jnts: %d)\n",str.c_str(),stream_nJoints,(int)cmdVector.size());
 *        return;
 *    }
 *    if (cmdVector.data()==0)
 *    {
 *         yError("Received null reference vector");
 *         return;
 *    }
 *
 *    switch (b.get(0).asVocab())
 *    {
 *        // manage commands with interface name as first
 *        case VOCAB_OPENLOOP_INTERFACE:
 *        {
 *            switch(b.get(1).asVocab())
 *            {
 *                case VOCAB_OPENLOOP_REF_OUTPUT:
 *                {
 *                    if (stream_IOpenLoop)
 *                    {
 *                        bool ok = stream_IOpenLoop->setRefOutput(b.get(2).asVocab(), cmdVector[0]);
 *                        if (!ok)
 *                            yError("Errors while trying to command an open loop message");
 *                    }
 *                    else
 *                        yError("OpenLoop interface not valid");
 *                }
 *                break;
 *
 *                case VOCAB_OPENLOOP_REF_OUTPUTS:
 *                {
 *                    if (stream_IOpenLoop)
 *                    {
 *                        bool ok=stream_IOpenLoop->setRefOutputs(cmdVector.data());
 *                        if (!ok)
 *                            yError("Errors while trying to command an open loop message");
 *                    }
 *                    else
 *                        yError("OpenLoop interface not valid\n");
 *                }
 *                break;
 *            }
 *            break;
 *        }
 *        break;
 *
 *        // fallback to commands without interface name
 *        case VOCAB_POSITION_MODE:
 *            {
 *                yError("Received VOCAB_POSITION_MODE this is an send invalid message on streaming port");
 *                break;
 *            }
 *        case VOCAB_POSITION_MOVES:
 *            {
 *                if (stream_IPosCtrl)
 *                    {
 *                        bool ok = stream_IPosCtrl->positionMove(cmdVector.data());
 *                        if (!ok)
 *                            yError("Errors while trying to start a position move");
 *                    }
 *
 *            }
 *            break;
 *
 *        case VOCAB_VELOCITY_MODE:
 *             {
 *                yError("Received VOCAB_VELOCITY_MODE this is an send invalid message on streaming port");
 *                break;
 *            }
 *
 *        case VOCAB_VELOCITY_MOVE:
 *            {
 *                stream_IVel->velocityMove(b.get(1).asInt(), cmdVector[0]);
 *            }
 *        break;
 *
 *        case VOCAB_VELOCITY_MOVES:
 *            {
 *                if (stream_IVel)
 *                    {
 *                        bool ok = stream_IVel->velocityMove(cmdVector.data());
 *                        if (!ok)
 *                            yError("Errors while trying to start a velocity move");
 *                    }
 *            }
 *            break;
 *
 *        case VOCAB_OUTPUTS:
 *            {
 *                yError() << "DEPRECATED openloop setOutputS!! missing interface name! Check you are using the updated RemoteControlBoard class "
 *                         << "Correct message should be [" << Vocab::decode(VOCAB_OPENLOOP_INTERFACE) << "] [" << Vocab::decode(VOCAB_OPENLOOP_REF_OUTPUTS) << "] list_if_values";
 *            }
 *            break;
 *
 *        case VOCAB_POSITION_DIRECT:
 *        {
 *            if(stream_IPosDirect)
 *            {
 *                bool ok = stream_IPosDirect->setPosition(b.get(1).asInt(), cmdVector[0]); // cmdVector.data());
 *                if (!ok)
 *                {   yError("Errors while trying to command an streaming position direct message on joint %d\n", b.get(1).asInt() ); }
 *            }
 *        }
 *        break;
 *
 *        case VOCAB_TORQUES_DIRECT:
 *        {
 *            if (stream_ITorque)
 *            {
 *                bool ok = stream_ITorque->setRefTorque(b.get(1).asInt(), cmdVector[0]);
 *                if (!ok)
 *                {   yError("Errors while trying to command a streaming torque direct message on all joints\n"); }
 *            }
 *        }
 *        break;
 *
 *        case VOCAB_TORQUES_DIRECTS:
 *        {
 *            if (stream_ITorque)
 *            {
 *                bool ok = stream_ITorque->setRefTorques(cmdVector.data());
 *                if (!ok)
 *                {   yError("Errors while trying to command a streaming torque direct message on all joints\n"); }
 *            }
 *        }
 *        break;
 *
 *        case VOCAB_TORQUES_DIRECT_GROUP:
 *        {
 *            if (stream_ITorque)
 *            {
 *                int n_joints = b.get(1).asInt();
 *                Bottle *jlut = b.get(2).asList();
 *                if( ((int)jlut->size() != n_joints) && ((int)cmdVector.size() != n_joints) )
 *                {
 *                    yError("Received VOCAB_TORQUES_DIRECT_GROUP size of joints vector or torques vector does not match the selected joint number\n" );
 *                }
 *
 *                int *joint_list = new int[n_joints];
 *                for (int i = 0; i < n_joints; i++)
 *                    joint_list[i] = jlut->get(i).asInt();
 *
 *
 *                bool ok = stream_ITorque->setRefTorques(n_joints, joint_list, cmdVector.data());
 *                if (!ok)
 *                {   yError("Error while trying to command a streaming toruqe direct message on joint group\n" ); }
 *
 *                delete[] joint_list;
 *            }
 *        }
 *        break;
 *
 *        case VOCAB_POSITION_DIRECT_GROUP:
 *        {
 *            if(stream_IPosDirect)
 *            {
 *                int n_joints = b.get(1).asInt();
 *                Bottle *jlut = b.get(2).asList();
 *                if( ((int)jlut->size() != n_joints) && ((int)cmdVector.size() != n_joints) )
 *                {
 *                    yError("Received VOCAB_POSITION_DIRECT_GROUP size of joints vector or positions vector does not match the selected joint number\n" );
 *                }
 *
 *                int *joint_list = new int[n_joints];
 *                for (int i = 0; i < n_joints; i++)
 *                    joint_list[i] = jlut->get(i).asInt();
 *
 *
 *                bool ok = stream_IPosDirect->setPositions(n_joints, joint_list, cmdVector.data());
 *                if (!ok)
 *                {   yError("Error while trying to command a streaming position direct message on joint group\n" ); }
 *
 *                delete[] joint_list;
 *            }
 *        }
 *        break;
 *
 *        case VOCAB_POSITION_DIRECTS:
 *        {
 *            if(stream_IPosDirect)
 *            {
 *                bool ok = stream_IPosDirect->setPositions(cmdVector.data());
 *                if (!ok)
 *                {   yError("Error while trying to command a streaming position direct message on all joints\n" ); }
 *            }
 *        }
 *        break;
 *        case VOCAB_VELOCITY_MOVE_GROUP:
 *        {
 *            if(stream_IVel2)
 *            {
 *                int n_joints = b.get(1).asInt();
 *                Bottle *jlut = b.get(2).asList();
 *                if( ((int)jlut->size() != n_joints) && ((int)cmdVector.size() != n_joints) )
 *                    yError("Received VOCAB_VELOCITY_MOVE_GROUP size of joints vector or positions vector does not match the selected joint number\n" );
 *
 *                int *joint_list = new int[n_joints];
 *                for (int i = 0; i < n_joints; i++)
 *                    joint_list[i] = jlut->get(i).asInt();
 *
 *                bool ok = stream_IVel2->velocityMove(n_joints, joint_list, cmdVector.data());
 *                if (!ok)
 *                {   yError("Error while trying to command a velocity move on joint group\n" ); }
 *
 *                delete[] joint_list;
 *            }
 *        }
 *        break;
 *
 *        default:
 *            {
 *                yarp::os::ConstString str = yarp::os::Vocab::decode(b.get(0).asVocab());
 *                yError("Unrecognized message while receiving on command port (%s)\n",str.c_str());
 *            }
 *            break;
 *        }
 */
}

#endif
