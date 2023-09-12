#pragma once

#include "FunctionProviderBase.hpp"

#include <uORB/topics/actuator_jets.h>

/**
 * Functions: Jet1 ... JetMax
 */
class FunctionJets: public FunctionProviderBase
{
public:
	static_assert(actuator_jets_s::NUM_CONTROLS == (int)OutputFunction::JetMax - (int)OutputFunction::Jet1 + 1,
		      "Unexpected num jets");

	FunctionJets(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_jets))
	{
		for (int i = 0; i < actuator_jets_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionJets(context); }

	void update() override { _topic.update(&_data); }
	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Jet1]; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	float defaultFailsafeValue(OutputFunction func) const override { return 0.f; }


private:
	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_jets_s _data{};
};
