//
// Created by balazs on 2021. 01. 31..
//

#ifndef LINK_INTERSECTION_BRUTE_FORCE_TYPEDLOG_H
#define LINK_INTERSECTION_BRUTE_FORCE_TYPEDLOG_H

#include "LogAppInterfaceStd.h"
#include "LogConverterCustomText.h"
#include "LogSenderRos2.h"
#include "LogQueueStdBoost.h"
#include "LogMessageCompact.h"
#include "Log.h"

namespace LogTopics {
extern nowtech::log::TopicInstance system;
extern nowtech::log::TopicInstance dumpModel;
}

constexpr nowtech::log::TaskId cgMaxTaskCount = 23;
constexpr bool cgAllowRegistrationLog = true;
constexpr bool cgLogFromIsr = false;
constexpr size_t cgTaskShutdownSleepPeriod = 100u;
constexpr bool cgArchitecture64 = true;
constexpr uint8_t cgAppendStackBufferSize = 100u;
constexpr bool cgAppendBasePrefix = true;
constexpr bool cgAlignSigned = false;
using AtomicBufferType = int32_t;
constexpr size_t cgAtomicBufferExponent = 14u;
constexpr AtomicBufferType cgAtomicBufferInvalidValue = 1234546789;
constexpr size_t cgTransmitBufferSize = 65536u;
constexpr size_t cgPayloadSize = 32u;
constexpr bool cgSupportFloatingPoint = true;
constexpr size_t cgQueueSize = 4444u;
constexpr nowtech::log::LogTopic cgMaxTopicCount = 2;
constexpr nowtech::log::TaskRepresentation cgTaskRepresentation = nowtech::log::TaskRepresentation::cName;
constexpr size_t cgDirectBufferSize = 0u;
constexpr nowtech::log::ErrorLevel cgErrorLevel = nowtech::log::ErrorLevel::Info;

using LogAppInterface = nowtech::log::AppInterfaceStd<cgMaxTaskCount, cgLogFromIsr, cgTaskShutdownSleepPeriod>;
constexpr typename LogAppInterface::LogTime cgTimeout = 123u;
constexpr typename LogAppInterface::LogTime cgRefreshPeriod = 444;
constexpr int cgSimulatedRos2Loglevel = 50;
using LogMessage = nowtech::log::MessageCompact<cgPayloadSize, cgSupportFloatingPoint>;
using LogConverter = nowtech::log::ConverterCustomText<LogMessage, cgArchitecture64, cgAppendStackBufferSize, cgAppendBasePrefix, cgAlignSigned>;
using LogSender = nowtech::log::SenderRos2<LogAppInterface, LogConverter, cgTransmitBufferSize, cgTimeout, cgSimulatedRos2Loglevel>;
using LogQueue = nowtech::log::QueueStdBoost<LogMessage, LogAppInterface, cgQueueSize>;
using LogAtomicBuffer = nowtech::log::AtomicBufferOperational<LogAppInterface, AtomicBufferType, cgAtomicBufferExponent, cgAtomicBufferInvalidValue>;
using LogConfig = nowtech::log::Config<cgAllowRegistrationLog, cgMaxTopicCount, cgTaskRepresentation, cgDirectBufferSize, cgRefreshPeriod, cgErrorLevel>;
using Log = nowtech::log::Log<LogQueue, LogSender, LogAtomicBuffer, LogConfig>;

#endif //LINK_INTERSECTION_BRUTE_FORCE_TYPEDLOG_H
