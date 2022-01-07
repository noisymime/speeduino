//#include <Arduino.h>
#include <string.h> // memcpy
#include <unity.h>
#include <stdio.h>
#include "tests_maths.h"
#include "maths.h"


void testMaths()
{
  RUN_TEST(test_maths_percent_U8);
  RUN_TEST(test_maths_percent_U16);
  RUN_TEST(test_maths_percent_U32);
  RUN_TEST(test_maths_halfpercent_U8);
  RUN_TEST(test_maths_halfpercent_U16);
  RUN_TEST(test_maths_halfpercent_U32);
  RUN_TEST(test_maths_div100_U8);
  RUN_TEST(test_maths_div100_U16);
  RUN_TEST(test_maths_div100_U32);
  RUN_TEST(test_maths_div100_S8);
  RUN_TEST(test_maths_div100_S16);
  RUN_TEST(test_maths_div100_S32);
  RUN_TEST(test_maths_div10_U8);
  RUN_TEST(test_maths_div10_U16);
  RUN_TEST(test_maths_div10_U32);
  
}

void test_maths_percent_U8(void)
{
  uint8_t percentOf = 200;
  TEST_ASSERT_EQUAL(percentage(50,  percentOf), 100);
  TEST_ASSERT_EQUAL(percentage(75,  percentOf), 150);
  TEST_ASSERT_EQUAL(percentage(0,   percentOf), 0);
  TEST_ASSERT_EQUAL(percentage(100, percentOf), 200);
  TEST_ASSERT_EQUAL(percentage(125, percentOf), 250);
}

void test_maths_percent_U16(void)
{
  uint16_t percentOf = 20000;
  TEST_ASSERT_EQUAL(percentage(50,  percentOf), 10000);
  TEST_ASSERT_EQUAL(percentage(75,  percentOf), 15000);
  TEST_ASSERT_EQUAL(percentage(0,   percentOf), 0);
  TEST_ASSERT_EQUAL(percentage(100, percentOf), 20000);
  TEST_ASSERT_EQUAL(percentage(125, percentOf), 25000);
}

void test_maths_percent_U32(void)
{
  uint32_t percentOf = 20000000UL;
  TEST_ASSERT_EQUAL(percentage(50, percentOf), 10000000UL);
  TEST_ASSERT_EQUAL(percentage(75, percentOf), 15000000UL);
  TEST_ASSERT_EQUAL(percentage(0, percentOf), 0);
  TEST_ASSERT_EQUAL(percentage(100, percentOf), 20000000UL);
  TEST_ASSERT_EQUAL(percentage(125, percentOf), 25000000UL);
}

void test_maths_halfpercent_U8(void)
{
  uint8_t percentOf = 200;
  TEST_ASSERT_EQUAL(halfPercentage(50, percentOf), 50);
  TEST_ASSERT_EQUAL(halfPercentage(75, percentOf), 75);
  TEST_ASSERT_EQUAL(halfPercentage(0, percentOf), 0);
  TEST_ASSERT_EQUAL(halfPercentage(100, percentOf), 100);
  TEST_ASSERT_EQUAL(halfPercentage(125, percentOf), 125);
}

void test_maths_halfpercent_U16(void)
{
  uint16_t percentOf = 20000;
  TEST_ASSERT_EQUAL(halfPercentage(50, percentOf), 5000);
  TEST_ASSERT_EQUAL(halfPercentage(75, percentOf), 7500);
  TEST_ASSERT_EQUAL(halfPercentage(0, percentOf), 0);
  TEST_ASSERT_EQUAL(halfPercentage(100, percentOf), 10000);
  TEST_ASSERT_EQUAL(halfPercentage(125, percentOf), 12500);
}

void test_maths_halfpercent_U32(void)
{
  uint32_t percentOf = 20000000UL;
  TEST_ASSERT_EQUAL(halfPercentage(50, percentOf), 5000000UL);
  TEST_ASSERT_EQUAL(halfPercentage(75, percentOf), 7500000UL);
  TEST_ASSERT_EQUAL(halfPercentage(0, percentOf), 0);
  TEST_ASSERT_EQUAL(halfPercentage(100, percentOf), 10000000UL);
  TEST_ASSERT_EQUAL(halfPercentage(125, percentOf), 12500000UL);
}

void test_maths_div100_U8(void)
{
  TEST_ASSERT_EQUAL(divu100(100), 1);
  TEST_ASSERT_EQUAL(divu100(200), 2);
  TEST_ASSERT_EQUAL(divu100(0), 0);
  TEST_ASSERT_EQUAL(divu100(50), 0);
  TEST_ASSERT_EQUAL(divu100(250), 2);
}

void test_maths_div100_U16(void)
{
  TEST_ASSERT_EQUAL(divu100(10000), 100);
  TEST_ASSERT_EQUAL(divu100(40000), 400);
}

void test_maths_div100_U32(void)
{
  TEST_ASSERT_EQUAL(divu100(100000000UL), 1000000UL);
  TEST_ASSERT_EQUAL(divu100(200000000UL), 2000000UL);
}

void test_maths_div100_S8(void)
{
  //Check both the signed and unsigned results
  TEST_ASSERT_EQUAL(divs100(100), 1);
  TEST_ASSERT_EQUAL(divs100(0), 0);
  TEST_ASSERT_EQUAL(divs100(50), 0);

  TEST_ASSERT_EQUAL(divs100(-100), -1);
  TEST_ASSERT_EQUAL(divs100(-50), 0);
  TEST_ASSERT_EQUAL(divs100(-120), -1);
}

void test_maths_div100_S16(void)
{
  //Check both the signed and unsigned results
  TEST_ASSERT_EQUAL(divs100(10000), 100);
  TEST_ASSERT_EQUAL(divs100(0), 0);
  TEST_ASSERT_EQUAL(divs100(50), 0);

  TEST_ASSERT_EQUAL(divs100(-10000), -100);
  TEST_ASSERT_EQUAL(divs100(-50), 0);
  TEST_ASSERT_EQUAL(divs100(-120), -1);
}

void test_maths_div100_S32(void)
{
  //Check both the signed and unsigned results
  TEST_ASSERT_EQUAL(divs100(100000000L), 1000000L);
  TEST_ASSERT_EQUAL(divs100(0), 0);
  TEST_ASSERT_EQUAL(divs100(50), 0);

  TEST_ASSERT_EQUAL(divs100(-100000000L), -1000000L);
  TEST_ASSERT_EQUAL(divs100(-50), 0);
  TEST_ASSERT_EQUAL(divs100(-120), -1);
}

void test_maths_div10_U8(void)
{
  TEST_ASSERT_EQUAL(divu10(100), 10);
  TEST_ASSERT_EQUAL(divu10(200), 20);
  TEST_ASSERT_EQUAL(divu10(0), 0);
  TEST_ASSERT_EQUAL(divu10(50), 5);
  TEST_ASSERT_EQUAL(divu10(250), 25);
}

void test_maths_div10_U16(void)
{
  TEST_ASSERT_EQUAL(divu10(10000), 1000);
  TEST_ASSERT_EQUAL(divu10(40000), 4000);
  TEST_ASSERT_EQUAL(divu10(0), 0);
  TEST_ASSERT_EQUAL(divu10(50), 5);
  TEST_ASSERT_EQUAL(divu10(250), 25);
}

void test_maths_div10_U32(void)
{
  TEST_ASSERT_EQUAL(divu10(100000000UL), 10000000UL);
  TEST_ASSERT_EQUAL(divu10(400000000UL), 40000000UL);
  TEST_ASSERT_EQUAL(divu10(0), 0);
  TEST_ASSERT_EQUAL(divu10(50), 5);
  TEST_ASSERT_EQUAL(divu10(250), 25);
}
