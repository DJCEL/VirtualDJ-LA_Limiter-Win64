#include "LA_Limiter.h"

//----------------------------------------------------------------------------
CLA_Limiter::CLA_Limiter()
{
    threshold = 0.0f;
    output = 0.0f;
    is_TruePeak = 0;
    threshold_db = 0.0f;
    output_db = 0.0f;
    m_isOn = 0;
    nbOn = 0;
}
//----------------------------------------------------------------------------
CLA_Limiter::~CLA_Limiter()
{

}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnLoad()
{
    DeclareParameterSlider(&threshold,ID_SLIDER_1,"Threshold","THR",0.0f);
    DeclareParameterSlider(&output, ID_SLIDER_2,"Output","OUT",0.0f);
    DeclareParameterSwitch(&is_TruePeak, ID_SWITCH_1,"True Peak","ISP",false);
    DeclareParameterSwitch(&m_isOn, ID_SWITCH_2, "O", "O", false);

    OnParameter(ID_INIT);
    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnGetPluginInfo(TVdjPluginInfo8* infos)
{
    infos->PluginName = "LA_Limiter";
    infos->Author = "DJ CEL";
    infos->Description = "Look-ahead stereo limiter";
    infos->Version = "1.0.0";
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
        OnButton(ID_SWITCH_1);
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
            threshold_db = -30.0f * threshold;
            limiter.setThreshold(threshold_db);
		    break;
	    case ID_SLIDER_2:
            output_db = -30.0f * output;
            limiter.setOutput(output_db);
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
    }

    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnStart()
{
    m_isOn = 0;
    limiter.prepare(SampleRate);
    OnParameter(ID_INIT);
    limiter.reset();
    return S_OK;
}
//----------------------------------------------------------------------------
HRESULT VDJ_API CLA_Limiter::OnStop()
{
    limiter.reset();
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
