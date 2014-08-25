#ifndef KFBUILDER_H
#define KFBUILDER_H
#include "IWeightedKalmanFilter.h"

class KFBuilder
{
public:
    enum Model
    {
        kmobile_object_model,
        krobot_model,
        ktotal_models
    };
    enum Filter
    {
        kseq_ukf_filter,
        ksr_seq_ukf_filter,
        kbasic_ukf_filter,
        ksr_basic_ukf_filter,
        ktotal_filters
    };
    KFBuilder();
    static IWeightedKalmanFilter* getNewFilter(Filter filter_type, Model model_type);
};

std::string FilterTypeString(KFBuilder::Filter filter_type);

#endif // KFBUILDER_H
