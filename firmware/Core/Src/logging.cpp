#include "logging.h"

void logging::init(){
    // nothing
}

message_severities logging::get_active_severity(void){
    return active_severity;
}

uint64_t logging::get_last_reset_time_millis(void){
    return last_reset_time_millis;
}

uint64_t logging::get_last_active_error_time_millis(uint32_t id){
    return last_active_error_times_millis[uint32_t(id)];
}

void logging::clear_all(void){
    for (uint32_t i = 0; i < identifiers_count; i++){
        last_active_error_times_millis[i] = 0;  // reset message times
        last_ok_error_times_millis[i] = *milliseconds;  // update ok times to now
    }

    active_severity = message_severities(0);   // reset active severity
    last_reset_time_millis = *milliseconds;    // update last reset time
}

message_severities logging::add(uint32_t  id){
    last_active_error_times_millis[uint32_t(id)] = *milliseconds;  // update error time
    active_severity = message_severities(values[id] >> 28);   // update active severity
    
    return active_severity;
}

message_severities logging::log_persistent_active(uint32_t id){
    if(last_ok_error_times_millis[uint32_t(id)] + persistent_error_trigger_times_millis[uint32_t(id)] >= *milliseconds){
        // the id has been active for long enough
        add(id);
    }
    return active_severity;
}

message_severities logging::log_persistent_inactive(uint32_t id){
    last_ok_error_times_millis[uint32_t(id)] = *milliseconds;  // update ok time
    return active_severity;
}

void logging::set_persistent_error_trigger_time(uint32_t id, uint32_t time){
    persistent_error_trigger_times_millis[uint32_t(id)] = time;
}

