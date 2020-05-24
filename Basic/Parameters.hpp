/*
	参数接口
	朱文杰20191001
	注意：仅可在内部存储可用后调用此接口
*/

#pragma once

#include <stdint.h>
#include "mavlink.h"
#include "Basic.hpp"

enum PR_RESULT
{
	PR_OK = 0 ,
	PR_Existed ,
	PR_NewParam ,
	PR_TimeOut ,
	PR_ERR ,
};

/*
	参数组注册（所有名称小于等于16字节）
	注意：仅可在初始化完成前调用（setInitializationCompleted前）
	name：参数组名称
	param_version：参数版本
	params_count：参数个数

	返回值：
	PR_ERR：参数表初始化未完成
	PR_Existed: 参数表中已有同名参数
	PR_TimeOut：访问参数表操作超时
	PR_NewParam：注册的参数是新参数（存储器里没有）
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count,
															const MAV_PARAM_TYPE param_types[], const SName param_names[], const uint64_t initial_params[] );

/*
	参数组注册（所有名称小于等于16字节）
	注意：此函数只能注册已在存储器中的参数
	注意：仅可在参数表初始化完成前调用
	name：参数组名称
	param_version：参数版本
	params_count：参数个数n

	param_names：参数名称*n（可以为NULL代表不需要加入参数列表）

	返回值：
	PR_ERR：参数表初始化未完成或参数在存储器中不存在
	PR_Existed: 参数表中已有同名参数
	PR_TimeOut：访问参数表操作超时
	PR_NewParam：注册的参数是新参数（存储器里没有）
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count, const SName param_names[] );

/*
	从存储器重新载入参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称

	返回值：
	PR_ERR：参数表初始化未完成或参数在存储器中不存在
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT ReloadParamGroup( SName name );

/*
	获取参数表参数个数
	注意：仅可在参数表初始化完成后调用
	count：返回参数个数
返回值：
	PR_ERR：未完成初始化
	PR_OK：读取成功
*/
PR_RESULT GetParametersCount( uint32_t* count );
/*
	重置参数表读取迭代器
	注意：仅可在参数表初始化完成后调用
	count：返回参数个数
返回值：
	PR_ERR：未完成初始化
	PR_OK：操作成功
*/
PR_RESULT ResetParametersIterator();
/*
	递增迭代器
	注意：仅可在参数表初始化完成后调用
返回值：
	PR_ERR：未完成初始化或已经在末尾
	PR_OK：操作成功
*/
PR_RESULT ParameterIteratorMoveNext();
/*
	读当前参数
	注意：仅可在参数表初始化完成后调用	
	name：读取的参数名称
	index：当前参数序号
	type：读取的参数类型
	value：读取的参数的值
	is_new：参数是为新（不在存储器）
返回值：
	PR_ERR：未完成初始化或已经在末尾
	PR_TimeOut：访问参数表操作超时
	PR_OK：操作成功
*/
PR_RESULT ReadCurrentParameter( SName* name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );

/*
	读取指定名称参数
	注意：仅可在参数表初始化完成后调用
	name：参数名称
	type：读取的参数类型
	value：要读取的参数的值
	is_new：参数是为新（不在存储器）

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParam( SName name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );
/*
	读取指定序号参数（低效率）
	注意：仅可在参数表初始化完成后调用
	index：参数序号
	name：参数名称
	type：读取的参数类型
	value：要读取的参数的值
	is_new：参数是为新（不在存储器）

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParam( uint32_t index, SName* name, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT = -1 );

/*
	读取指定名称参数组
	注意：仅可在参数表初始化完成后调用（setInitializationCompleted后）
	name：参数组名称
	data：要读取的参数组参数的值

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, double TIMEOUT = -1 );
/*
	读取指定名称参数组
	注意：仅可在参数表初始化完成后调用（setInitializationCompleted后）
	name：参数组名称
	data：要读取的参数组参数的值
	start：从序号start开始读取（0开始）
	read_count：读取数目

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, uint16_t start, uint16_t read_count, double TIMEOUT = -1 );

/*
	更新指定名称参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要更新的参数组参数的值
	start：从序号start开始更新（0开始）
	write_count：更新数目
	st：是否写入存储器
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT UpdateParamGroup( SName name, const uint64_t data[], uint16_t start, uint16_t write_count, bool st = true, double TIMEOUT = -1 );

/*
	将指定名称参数组名称的参数保存到存储器
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：保存成功
*/
PR_RESULT SaveParamGroup( SName name, double TIMEOUT = -1 );

/*
	更新指定名称参数
	注意：仅可在参数表初始化完成后调用（setInitializationCompleted后）
	name：参数组名称
	data：要更新的参数组参数的值
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：更新成功
*/
PR_RESULT UpdateParam( SName name, const uint64_t data, double TIMEOUT = -1 );

void init_Parameters();