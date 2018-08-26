#ifndef __POWER_ZTE_MISC__
#define __POWER_ZTE_MISC__

enum battery_sts {
	BATTERY_CHARGING = 0,
	BATTERY_DISCHARGING,/*set icl min value and disable charger*/
	BATTERY_NOT_CHARGING,/*only disable charger*/
	BATTERY_UNKNOWN
};

enum charging_policy_sts {
	NORMAL_CHARGING_POLICY = BIT(0),
	DEMO_CHARGING_POLICY = BIT(1),
	EXPIRED_CHARGING_POLICY = BIT(2),/*depends on EXPIRED_CHARGING_POLICY_ENABLE*/
	UNKNOWN_CHARGING_POLICY = BIT(3)
};

#define DEFAULT_CHARGING_POLICY NORMAL_CHARGING_POLICY
#define MIN_BATTERY_PROTECTED_PERCENT 50
#define MAX_BATTERY_PROTECTED_PERCENT 70

#if (defined(CONFIG_BOARD_LENA) || defined(CONFIG_BOARD_CASTELLA))
#define EXPIRED_CHARGING_POLICY_ENABLE 1
#else
#define EXPIRED_CHARGING_POLICY_ENABLE 0
#endif

#define CHARGING_EXPIRATION_TIME_NS 86400000000000

struct charging_policy_ops {
	int battery_status;
	int charging_policy_status;
	int (*charging_policy_demo_sts_set)(struct charging_policy_ops *charging_policy, bool enable);
	int (*charging_policy_demo_sts_get)(struct charging_policy_ops *charging_policy);
	int (*charging_policy_expired_sts_get)(struct charging_policy_ops *charging_policy);
	int (*charging_policy_expired_sec_set)(struct charging_policy_ops *charging_policy, int sec);
	int (*charging_policy_expired_sec_get)(struct charging_policy_ops *charging_policy);
};

int zte_misc_register_charging_policy_ops(struct charging_policy_ops *ops);

#endif
