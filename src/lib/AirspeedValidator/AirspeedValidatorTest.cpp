/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#define private public
#include "AirspeedValidator.hpp"


// to run: make tests TESTFILTER=AirspeedValidator



class AirspeedValidatorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// param_reset_all();
	}
};


class TestAirspeedValidator : public AirspeedValidator
{
public:
	TestAirspeedValidator() : AirspeedValidator() {}
};


TEST_F(AirspeedValidatorTest, testLoadFactorCheck)
{
	// GIVEN a parameter handle
	TestAirspeedValidator av;
	float accel_z = 10.0f;
	float load_factor_ratio_prior = 0.5f;
	av._in_fixed_wing_flight = true;
	av._EAS = 10.0f;
	av._airspeed_stall = 20.0f;
	av._load_factor_ratio = load_factor_ratio_prior;

	// WHEN: we get the parameter
	av.check_load_factor(accel_z);

	// THEN it should be successful and have the default value
	EXPECT_GT(av._load_factor_ratio, load_factor_ratio_prior);
}


//
// TEST_F(AirspeedValidatorTest, testEAsTAScalculation)
// {
// 	// GIVEN a parameter handle
//
// 	// WHEN: we get the parameter
//
// 	// THEN it should be successful and have the default value
//
//   int a = 1;
//   int b = 1;
//
//   EXPECT_EQ(a,b);
//
//   // GIVEN
//   TestAirspeedValidator av;
//
//   // av._EAS_scale = 1.0f;
//   // av._IAS = 10.0f;
//   // float air_pressure_pa = 101325.0f;
//   // float air_temperature_celsius = 20.0f;
//
//   // WHEN
//   // av.update_EAS_TAS(air_pressure_pa, air_temperature_celsius);
//   calc_EAS_from_IAS(10.0f, 1.0f);
//
//   // THEN
//   // EXPECT_LT(abs(av._EAS), 0.1);
//
//
//
//   // bool _in_fixed_wing_flight = false;
//
//
// }
