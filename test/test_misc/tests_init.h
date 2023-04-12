#define UNKNOWN_PIN 0xFF

extern void testInitialisation();
void test_initialisation_complete(void);
void test_initialisation_ports(void);
void test_initialisation_outputs_V03(void);
void test_initialisation_outputs_V04(void);
void test_initialisation_outputs_MX5_8995(void);
void test_initialisation_outputs_PWM_idle(void);
void test_initialisation_outputs_stepper_idle(void);
void test_initialisation_outputs_boost(void);
void test_initialisation_outputs_VVT(void);
uint8_t getPinMode(uint8_t);