#pragma once

#include "vdjDsp8.h"
#include "Limiter.h"
#include <cstdio>
#include <cstring>

class CLA_Limiter : public IVdjPluginDsp8
{
    public:
        CLA_Limiter();
        ~CLA_Limiter();
        HRESULT VDJ_API OnLoad() override;
        HRESULT VDJ_API OnGetPluginInfo(TVdjPluginInfo8* infos) override;
        ULONG   VDJ_API Release() override;
        HRESULT VDJ_API OnStart() override;
        HRESULT VDJ_API OnStop() override;
        HRESULT VDJ_API OnParameter(int id) override;
        HRESULT VDJ_API OnGetParameterString(int id, char* outParam, int outParamSize) override;
        HRESULT VDJ_API OnProcessSamples(float* buffer, int nb) override;

    private:
        typedef enum _Interface
        {
            ID_INIT,
            ID_SLIDER_1,
            ID_SLIDER_2,
            ID_SWITCH_1,
            ID_SWITCH_2
        } ID_Interface;

        void OnSlider(int id);
        void OnButton(int id);

        float threshold;
        float output;
        int is_TruePeak;
        float threshold_db;  
        float output_db;
        int m_isOn;
        int nbOn;
        Limiter limiter;
};