#include "LA_Limiter.h"

//----------------------------------------------------------------------------
CLA_Limiter::CLA_Limiter()
{
    is_TruePeak = 0;
    is_FinalSecurity = 0;
    threshold_db = 0.0f;
    output_db = 0.0f;
    releaseMs = 0.0f;
    holdMs = 0.0f;
    m_isOn = 0;
    nbOn = 0;
    memset(SliderValue, 0, 4 * sizeof(float));
}
//----------------------------------------------------------------------------
CLA_Limiter::~CLA_Limiter()
{

}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnLoad()
{
    DeclareParameterSlider(&SliderValue[0], ID_SLIDER_1, "Threshold", "THR", 0.0f);
    DeclareParameterSlider(&SliderValue[1], ID_SLIDER_2,"Output","OUT",0.0f);
    DeclareParameterSlider(&SliderValue[2], ID_SLIDER_3, "releaseMs", "REL", 0.146f);
    DeclareParameterSlider(&SliderValue[3], ID_SLIDER_4, "holdMs", "HOL", 0.3f);
    DeclareParameterSwitch(&is_TruePeak, ID_SWITCH_1,"True Peak","ISP",false);
    DeclareParameterSwitch(&is_FinalSecurity, ID_SWITCH_2, "Final Security", "SEC", false);
    DeclareParameterSwitch(&m_isOn, ID_SWITCH_3, "O", "O", false);

    OnParameter(ID_INIT);
    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnGetPluginInfo(TVdjPluginInfo8* infos)
{
    infos->PluginName = "LA_Limiter";
    infos->Author = "DJ CEL";
    infos->Description = "Look-ahead stereo limiter";
    infos->Version = "2.0.3";
    infos->Flags = 0x00;
    infos->Bitmap = NULL;
    return S_OK;
}
//----------------------------------------------------------------------------
ULONG VDJ_API CLA_Limiter::Release()
{
    m_isOn = 0;
    delete this;
    return 0;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnParameter(int id)
{
    if (id == ID_INIT)
    {
        OnSlider(ID_SLIDER_1);
        OnSlider(ID_SLIDER_2);
        OnSlider(ID_SLIDER_3);
        OnSlider(ID_SLIDER_4);
        OnButton(ID_SWITCH_1);
        OnButton(ID_SWITCH_2);
    }
    else
    {
        OnSlider(id);
        OnButton(id);
    }

    return S_OK;
}
//------------------------------------------------------------------------------
void CLA_Limiter::OnSlider(int id)
{
    switch (id)
    {
	    case ID_SLIDER_1:
            threshold_db = -30.0f * SliderValue[0];
            limiter.setThreshold(threshold_db, -30.0f, 0.0f);
		    break;
	    case ID_SLIDER_2:
            output_db = -30.0f * SliderValue[1];
            limiter.setOutput(output_db, -30.0f, 0.0f);
		    break;
        case ID_SLIDER_3:
            releaseMs = 5.0f + SliderValue[2] * (1000.0f - 5.0f);
			limiter.setReleaseMs(releaseMs, 5.0f, 1000.0f);
            break;
        case ID_SLIDER_4:
            holdMs = SliderValue[3] * 100.0f;
            limiter.setHoldMs(holdMs, 0.0f, 100.0f);
            break;
    }
}
//------------------------------------------------------------------------------
void CLA_Limiter::OnButton(int id)
{
    switch(id)
	{
	    case ID_SWITCH_1:
            limiter.setTruePeak(is_TruePeak != 0);
		    break;
        case ID_SWITCH_2:
            limiter.setFinalSecurity(is_FinalSecurity != 0);
            break;
	}
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnGetParameterString(int id,char* outParam,int outParamSize)
{
    switch (id)
    {
        case ID_SLIDER_1:
            std::snprintf(outParam,outParamSize,"%.1f dB", threshold_db);
            break;

        case ID_SLIDER_2:
            std::snprintf(outParam,outParamSize,"%.1f dB", output_db);
            break;

        case ID_SLIDER_3:
            std::snprintf(outParam, outParamSize, "%.0f ms", releaseMs);
            break;

        case ID_SLIDER_4:
            std::snprintf(outParam, outParamSize, "%.0f ms", holdMs);
            break;
    }

    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnStart()
{
    m_isOn = 0;
    limiter.start(SampleRate);
    OnParameter(ID_INIT);
    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnStop()
{
    limiter.stop();
    m_isOn = 0;
    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnProcessSamples(float* buffer, int nb)
{
    limiter.process(buffer, nb);
	nbOn = limiter.isActive();
    m_isOn = (nbOn > 0);
    return S_OK;
}
