#pragma once
#ifdef ROSCPP_ROS_H
#include <stdint.h>
#include <string>
#include <vector>
#include "ros/ros.h"
#include <network_tables_node/NTGetBool.h>
#include <network_tables_node/NTGetBoolArray.h>
#include <network_tables_node/NTGetDouble.h>
#include <network_tables_node/NTGetDoubleArray.h>
#include <network_tables_node/NTGetRaw.h>
#include <network_tables_node/NTGetString.h>
#include <network_tables_node/NTGetStringArray.h>
#include <network_tables_node/NTSetBool.h>
#include <network_tables_node/NTSetBoolArray.h>
#include <network_tables_node/NTSetDouble.h>
#include <network_tables_node/NTSetDoubleArray.h>
#include <network_tables_node/NTSetRaw.h>
#include <network_tables_node/NTSetString.h>
#include <network_tables_node/NTSetStringArray.h>

extern ros::NodeHandle* node;

namespace ck
{
    namespace nt
    {
        namespace internal
        {
            static ros::ServiceClient __nt_setbool_client;
            static ros::ServiceClient __nt_setboolarray_client;
            static ros::ServiceClient __nt_setdouble_client;
            static ros::ServiceClient __nt_setdoublearray_client;
            static ros::ServiceClient __nt_setstring_client;
            static ros::ServiceClient __nt_setstringarray_client;
            static ros::ServiceClient __nt_setraw_client;

            static ros::ServiceClient __nt_getbool_client;
            static ros::ServiceClient __nt_getboolarray_client;
            static ros::ServiceClient __nt_getdouble_client;
            static ros::ServiceClient __nt_getdoublearray_client;
            static ros::ServiceClient __nt_getstring_client;
            static ros::ServiceClient __nt_getstringarray_client;
            static ros::ServiceClient __nt_getraw_client;


            ////////////////////////////////////////////////////////////////////////////////////////
            //Set Services
            ////////////////////////////////////////////////////////////////////////////////////////
            static ros::ServiceClient& __set_nt_bool_srv_get()
            {
                if (node && !__nt_setbool_client)
                {
                    __nt_setbool_client = node->serviceClient<network_tables_node::NTSetBool>("nt_setbool", true);
                }
                return __nt_setbool_client;
            };

            static ros::ServiceClient& __set_nt_bool_array_srv_get()
            {
                if (node && !__nt_setboolarray_client)
                {
                    __nt_setboolarray_client = node->serviceClient<network_tables_node::NTSetBoolArray>("nt_setboolarray", true);
                }
                return __nt_setboolarray_client;
            };

            static ros::ServiceClient& __set_nt_double_srv_get()
            {
                if (node && !__nt_setdouble_client)
                {
                    __nt_setdouble_client = node->serviceClient<network_tables_node::NTSetDouble>("nt_setdouble", true);
                }
                return __nt_setdouble_client;
            };

            static ros::ServiceClient& __set_nt_double_array_srv_get()
            {
                if (node && !__nt_setdoublearray_client)
                {
                    __nt_setdoublearray_client = node->serviceClient<network_tables_node::NTSetDoubleArray>("nt_setdoublearray", true);
                }
                return __nt_setdoublearray_client;
            };

            static ros::ServiceClient& __set_nt_string_srv_get()
            {
                if (node && !__nt_setstring_client)
                {
                    __nt_setstring_client = node->serviceClient<network_tables_node::NTSetString>("nt_setstring", true);
                }
                return __nt_setstring_client;
            };

            static ros::ServiceClient& __set_nt_string_array_srv_get()
            {
                if (node && !__nt_setstringarray_client)
                {
                    __nt_setstringarray_client = node->serviceClient<network_tables_node::NTSetStringArray>("nt_setstringarray", true);
                }
                return __nt_setstringarray_client;
            };

            [[maybe_unused]]
            static ros::ServiceClient& __set_nt_raw_srv_get()
            {
                if (node && !__nt_setraw_client)
                {
                    __nt_setraw_client = node->serviceClient<network_tables_node::NTSetRaw>("nt_setraw", true);
                }
                return __nt_setraw_client;
            };




            ////////////////////////////////////////////////////////////////////////////////////////
            //Get Services
            ////////////////////////////////////////////////////////////////////////////////////////

            static ros::ServiceClient& __get_nt_bool_srv_get()
            {
                if (node && !__nt_getbool_client)
                {
                    __nt_getbool_client = node->serviceClient<network_tables_node::NTGetBool>("nt_getbool", true);
                }
                return __nt_getbool_client;
            };

            static ros::ServiceClient& __get_nt_bool_array_srv_get()
            {
                if (node && !__nt_getboolarray_client)
                {
                    __nt_getboolarray_client = node->serviceClient<network_tables_node::NTGetBoolArray>("nt_getboolarray", true);
                }
                return __nt_getboolarray_client;
            };

            static ros::ServiceClient& __get_nt_double_srv_get()
            {
                if (node && !__nt_getdouble_client)
                {
                    __nt_getdouble_client = node->serviceClient<network_tables_node::NTGetDouble>("nt_getdouble", true);
                }
                return __nt_getdouble_client;
            };

            static ros::ServiceClient& __get_nt_double_array_srv_get()
            {
                if (node && !__nt_getdoublearray_client)
                {
                    __nt_getdoublearray_client = node->serviceClient<network_tables_node::NTGetDoubleArray>("nt_getdoublearray", true);
                }
                return __nt_getdoublearray_client;
            };

            static ros::ServiceClient& __get_nt_string_srv_get()
            {
                if (node && !__nt_getstring_client)
                {
                    __nt_getstring_client = node->serviceClient<network_tables_node::NTGetString>("nt_getstring", true);
                }
                return __nt_getstring_client;
            };

            static ros::ServiceClient& __get_nt_string_array_srv_get()
            {
                if (node && !__nt_getstringarray_client)
                {
                    __nt_getstringarray_client = node->serviceClient<network_tables_node::NTGetStringArray>("nt_getstringarray", true);
                }
                return __nt_getstringarray_client;
            };

            [[maybe_unused]]
            static ros::ServiceClient& __get_nt_raw_srv_get()
            {
                if (node && !__nt_getraw_client)
                {
                    __nt_getraw_client = node->serviceClient<network_tables_node::NTGetRaw>("nt_getraw", true);
                }
                return __nt_getraw_client;
            };
        };

        ////////////////////////////////////////////////////////////////////////////////////////
        //Set Methods
        ////////////////////////////////////////////////////////////////////////////////////////
        template <typename T>
        bool set(std::string table_name, std::string value_name, T value) = delete;

        template <>
        bool set<bool>(std::string table_name, std::string value_name, bool value)
        {
            ros::ServiceClient& nt_setbool_localclient = internal::__set_nt_bool_srv_get();
            if (nt_setbool_localclient)
            {
                network_tables_node::NTSetBool ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value;
                return nt_setbool_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<uint8_t>(std::string table_name, std::string value_name, uint8_t value)
        {
            ros::ServiceClient& nt_setbool_localclient = internal::__set_nt_bool_srv_get();
            if (nt_setbool_localclient)
            {
                network_tables_node::NTSetBool ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value > 0 ? true : false;
                return nt_setbool_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<double>(std::string table_name, std::string value_name, double value)
        {
            ros::ServiceClient& nt_setdouble_localclient = internal::__set_nt_double_srv_get();
            if (nt_setdouble_localclient)
            {
                network_tables_node::NTSetDouble ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value;
                return nt_setdouble_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<float>(std::string table_name, std::string value_name, float value)
        {
            ros::ServiceClient& nt_setdouble_localclient = internal::__set_nt_double_srv_get();
            if (nt_setdouble_localclient)
            {
                network_tables_node::NTSetDouble ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = (double)value;
                return nt_setdouble_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<std::string>(std::string table_name, std::string value_name, std::string value)
        {
            ros::ServiceClient& nt_setstring_localclient = internal::__set_nt_string_srv_get();
            if (nt_setstring_localclient)
            {
                network_tables_node::NTSetString ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value;
                return nt_setstring_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<std::vector<bool>>(std::string table_name, std::string value_name, std::vector<bool> value)
        {
            ros::ServiceClient& nt_setboolarray_localclient = internal::__set_nt_bool_array_srv_get();
            if (nt_setboolarray_localclient)
            {
                std::vector<uint8_t> tmpConvertVec;
                for (bool b : value)
                {
                    tmpConvertVec.push_back((uint8_t)(b));
                }
                network_tables_node::NTSetBoolArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = tmpConvertVec;
                return nt_setboolarray_localclient.call(ntmsg);
            }
            return false;
        };

        template <>
        bool set<std::vector<double>>(std::string table_name, std::string value_name, std::vector<double> value)
        {
            ros::ServiceClient& nt_setdoublearray_localclient = internal::__set_nt_double_array_srv_get();
            if (nt_setdoublearray_localclient)
            {
                network_tables_node::NTSetDoubleArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value;
                return nt_setdoublearray_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<std::vector<float>>(std::string table_name, std::string value_name, std::vector<float> value)
        {
            ros::ServiceClient& nt_setdoublearray_localclient = internal::__set_nt_double_array_srv_get();
            if (nt_setdoublearray_localclient)
            {
                network_tables_node::NTSetDoubleArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                std::vector<double> tmpConversion(value.begin(), value.end());
                ntmsg.request.value = tmpConversion;
                return nt_setdoublearray_localclient.call(ntmsg);

            }
            return false;
        };

        template <>
        bool set<std::vector<std::string>>(std::string table_name, std::string value_name, std::vector<std::string> value)
        {
            ros::ServiceClient& nt_setstringarray_localclient = internal::__set_nt_string_array_srv_get();
            if (nt_setstringarray_localclient)
            {
                network_tables_node::NTSetStringArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.value = value;
                return nt_setstringarray_localclient.call(ntmsg);

            }
            return false;
        };


        ////////////////////////////////////////////////////////////////////////////////////////
        //Get Methods
        ////////////////////////////////////////////////////////////////////////////////////////

        template <typename T>
        bool get(T& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, T default_value) = delete;

        template <>
        bool get<bool>(bool& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, bool default_value)
        {
            ros::ServiceClient& nt_getbool_localclient = internal::__get_nt_bool_srv_get();
            if (nt_getbool_localclient)
            {
                network_tables_node::NTGetBool ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getbool_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<uint8_t>(uint8_t& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, uint8_t default_value)
        {
            ros::ServiceClient& nt_getbool_localclient = internal::__get_nt_bool_srv_get();
            if (nt_getbool_localclient)
            {
                network_tables_node::NTGetBool ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getbool_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<float>(float& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, float default_value)
        {
            ros::ServiceClient& nt_getdouble_localclient = internal::__get_nt_double_srv_get();
            if (nt_getdouble_localclient)
            {
                network_tables_node::NTGetDouble ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getdouble_localclient.call(ntmsg);
                out = (float)ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<double>(double& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, double default_value)
        {
            ros::ServiceClient& nt_getdouble_localclient = internal::__get_nt_double_srv_get();
            if (nt_getdouble_localclient)
            {
                network_tables_node::NTGetDouble ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getdouble_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<std::string>(std::string& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, std::string default_value)
        {
            ros::ServiceClient& nt_getstring_localclient = internal::__get_nt_string_srv_get();
            if (nt_getstring_localclient)
            {
                network_tables_node::NTGetString ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getstring_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<std::vector<bool>>(std::vector<bool>& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, std::vector<bool> default_value)
        {
            ros::ServiceClient& nt_getboolarray_localclient = internal::__get_nt_bool_array_srv_get();
            if (nt_getboolarray_localclient)
            {
                std::vector<uint8_t> tmpConvertVec;
                for (bool b : default_value)
                {
                    tmpConvertVec.push_back((uint8_t)(b));
                }
                network_tables_node::NTGetBoolArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = tmpConvertVec;
                bool service_success = nt_getboolarray_localclient.call(ntmsg);
                if (service_success)
                {
                    out.clear();
                    for (bool b : ntmsg.response.output)
                    {
                        out.push_back((uint8_t)(b));
                    }
                }
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<std::vector<float>>(std::vector<float>& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, std::vector<float> default_value)
        {
            ros::ServiceClient& nt_getdoublearray_localclient = internal::__get_nt_double_array_srv_get();
            if (nt_getdoublearray_localclient)
            {
                network_tables_node::NTGetDoubleArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                std::vector<double> tmpConversion(default_value.begin(), default_value.end());
                ntmsg.request.default_value = tmpConversion;
                bool service_success = nt_getdoublearray_localclient.call(ntmsg);
                if (service_success)
                {
                    std::vector<float> tmpOut(ntmsg.response.output.begin(), ntmsg.response.output.end());
                    out = tmpOut;
                }
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<std::vector<double>>(std::vector<double>& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, std::vector<double> default_value)
        {
            ros::ServiceClient& nt_getdoublearray_localclient = internal::__get_nt_double_array_srv_get();
            if (nt_getdoublearray_localclient)
            {
                network_tables_node::NTGetDoubleArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getdoublearray_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };

        template <>
        bool get<std::vector<std::string>>(std::vector<std::string>& out, ros::Time& time_last_valid, std::string table_name, std::string value_name, std::vector<std::string> default_value)
        {
            ros::ServiceClient& nt_getstringarray_localclient = internal::__get_nt_string_array_srv_get();
            if (nt_getstringarray_localclient)
            {
                network_tables_node::NTGetStringArray ntmsg;
                ntmsg.request.table_name = table_name;
                ntmsg.request.entry_name = value_name;
                ntmsg.request.default_value = default_value;
                bool service_success = nt_getstringarray_localclient.call(ntmsg);
                out = ntmsg.response.output;
                time_last_valid = ntmsg.response.last_valid;
                return service_success;
            }
            time_last_valid = ros::Time(0);
            return false;
        };
    };
};
#endif