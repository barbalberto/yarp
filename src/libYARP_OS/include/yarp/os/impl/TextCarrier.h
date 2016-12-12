/*
 * Copyright (C) 2006 RobotCub Consortium
 * Authors: Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_OS_IMPL_TEXTCARRIER_H
#define YARP_OS_IMPL_TEXTCARRIER_H

#include <yarp/os/impl/TcpCarrier.h>

namespace yarp {
    namespace os {
        namespace impl {
            class TextCarrier;
        }
    }
}

/**
 * Communicating between two ports via a plain-text protocol.
 */
class yarp::os::impl::TextCarrier : public TcpCarrier
{

public:
    TextCarrier(bool ackVariant = false);

    virtual Carrier *create();

    virtual ConstString getName();

    virtual ConstString getSpecifierName();

    virtual bool checkHeader(const Bytes& header);
    virtual void getHeader(const Bytes& header);
    virtual bool requireAck();
    virtual bool isTextMode();
    virtual bool supportReply();
    virtual bool sendHeader(ConnectionState& proto);
    virtual bool expectReplyToHeader(ConnectionState& proto);
    virtual bool expectSenderSpecifier(ConnectionState& proto);
    virtual bool sendIndex(ConnectionState& proto, SizedWriter& writer);
    virtual bool expectIndex(ConnectionState& proto);
    virtual bool sendAck(ConnectionState& proto);
    virtual bool expectAck(ConnectionState& proto);
    virtual bool respondToHeader(ConnectionState& proto);
private:
    bool ackVariant;
};

#endif // YARP_OS_IMPL_TEXTCARRIER_H
