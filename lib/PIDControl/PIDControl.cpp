# 1 "./lib/PIDControl/PIDControl.cpp"
#include "PIDControl.h"
#include "Notifier.h"
#include "PIDSource.h"
#include "PIDOutput.h"
#include <math.h>
#include <vector>
#include "HAL/HAL.hpp"

static const std::string kP = "p";
static const std::string kI = "i";
static const std::string kD = "d";
static const std::string kF = "f";
static const std::string kSetpoint = "setpoint";
static const std::string kEnabled = "enabled";
static const std::string kIntegratedError = "integrated error";
# 28 "./lib/PIDControl/PIDControl.cpp"
PIDControl::PIDControl(float Kp, float Ki, float Kd, PIDSource *source,
  PIDOutput *output, float period) {
 Initialize(Kp, Ki, Kd, 0.0f, source, output, period);
}
# 44 "./lib/PIDControl/PIDControl.cpp"
PIDControl::PIDControl(float Kp, float Ki, float Kd, float Kf,
  PIDSource *source, PIDOutput *output, float period) {
 Initialize(Kp, Ki, Kd, Kf, source, output, period);
}

void PIDControl::Initialize(float Kp, float Ki, float Kd, float Kf,
  PIDSource *source, PIDOutput *output, float period) {
 m_controlLoop = std::make_unique<Notifier>(&PIDControl::Calculate, this);

 m_P = Kp;
 m_I = Ki;
 m_D = Kd;
 m_F = Kf;

 m_pidInput = source;
 m_pidOutput = output;
 m_period = period;

 m_controlLoop->StartPeriodic(m_period);
 m_setpointTimer.Start();

 static int32_t instances = 0;
 instances++;
 HALReport(HALUsageReporting::kResourceType_PIDController, instances);
}

PIDControl::~PIDControl() {
 if (m_table != nullptr)
  m_table->RemoveTableListener(this);
}





void PIDControl::Calculate() {
 bool enabled;
 PIDSource *pidInput;
 PIDOutput *pidOutput;

 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  pidInput = m_pidInput;
  pidOutput = m_pidOutput;
  enabled = m_enabled;
 }

 if (pidInput == nullptr)
  return;
 if (pidOutput == nullptr)
  return;

 if (enabled) {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  float input = pidInput->PIDGet();
  float result;
  PIDOutput *pidOutput;

  m_error = m_setpoint - input;
  if (m_continuous) {
   if (fabs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
    if (m_error > 0) {
     m_error = m_error - m_maximumInput + m_minimumInput;
    } else {
     m_error = m_error + m_maximumInput - m_minimumInput;
    }
   }
  }

  if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
   if (m_P != 0) {
    double potentialPGain = (m_totalError + m_error) * m_P;
    if (potentialPGain < m_maximumOutput) {
     if (potentialPGain > m_minimumOutput)
      m_totalError += m_error;
     else
      m_totalError = m_minimumOutput / m_P;
    } else {
     m_totalError = m_maximumOutput / m_P;
    }
   }

   m_result = m_D * m_error + m_P * m_totalError
     + CalculateFeedForward();
  } else {
   if (m_I != 0) {
    double potentialIGain = (m_totalError + m_error) * m_I;
    if (potentialIGain < m_maximumOutput) {
     if (potentialIGain > m_minimumOutput)
      m_totalError += m_error;
     else
      m_totalError = m_minimumOutput / m_I;
    } else {
     m_totalError = m_maximumOutput / m_I;
    }
   }
   else
   {
    if(m_totalError != 0)
     ResetIntegrator();
   }

   m_result = m_P * m_error + m_I * m_totalError
     + m_D * (m_error - m_prevError) + CalculateFeedForward();
  }
  m_prevError = m_error;

  if (m_result > m_maximumOutput)
   m_result = m_maximumOutput;
  else if (m_result < m_minimumOutput)
   m_result = m_minimumOutput;

  pidOutput = m_pidOutput;
  result = m_result;

  pidOutput->PIDWrite(m_inverted ? -result : result);


  m_buf.push(m_error);
  m_bufTotal += m_error;

  if (m_buf.size() > m_bufLength) {
   m_bufTotal -= m_buf.front();
   m_buf.pop();
  }
 }
}
# 187 "./lib/PIDControl/PIDControl.cpp"
double PIDControl::CalculateFeedForward() {
 if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
  return m_F * GetSetpoint();
 } else {
  double temp = m_F * GetDeltaSetpoint();
  m_prevSetpoint = m_setpoint;
  m_setpointTimer.Reset();
  return temp;
 }
}
# 205 "./lib/PIDControl/PIDControl.cpp"
void PIDControl::SetPID(double p, double i, double d) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_P = p;
  m_I = i;
  m_D = d;
 }

 if (m_table != nullptr) {
  m_table->PutNumber("p", m_P);
  m_table->PutNumber("i", m_I);
  m_table->PutNumber("d", m_D);
 }
}
# 228 "./lib/PIDControl/PIDControl.cpp"
void PIDControl::SetPID(double p, double i, double d, double f) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_P = p;
  m_I = i;
  m_D = d;
  m_F = f;
 }

 if (m_table != nullptr) {
  m_table->PutNumber("p", m_P);
  m_table->PutNumber("i", m_I);
  m_table->PutNumber("d", m_D);
  m_table->PutNumber("f", m_F);
 }
}





double PIDControl::GetP() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_P;
}





double PIDControl::GetI() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_I;
}





double PIDControl::GetD() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_D;
}





double PIDControl::GetF() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_F;
}






float PIDControl::Get() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_result;
}
# 298 "./lib/PIDControl/PIDControl.cpp"
void PIDControl::SetContinuous(bool continuous) {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 m_continuous = continuous;
}







void PIDControl::SetInputRange(float minimumInput, float maximumInput) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
 }

 SetSetpoint(m_setpoint);
}







void PIDControl::SetOutputRange(float minimumOutput, float maximumOutput) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_minimumOutput = minimumOutput;
  m_maximumOutput = maximumOutput;
 }
}






void PIDControl::SetSetpoint(float setpoint) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);

  if (m_maximumInput > m_minimumInput) {
   if (setpoint > m_maximumInput)
    m_setpoint = m_maximumInput;
   else if (setpoint < m_minimumInput)
    m_setpoint = m_minimumInput;
   else
    m_setpoint = setpoint;
  } else {
   m_setpoint = setpoint;
  }


  m_buf = std::queue<double>();
 }

 if (m_table != nullptr) {
  m_table->PutNumber("setpoint", m_setpoint);
 }
}





double PIDControl::GetSetpoint() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_setpoint;
}





double PIDControl::GetDeltaSetpoint() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return (m_setpoint - m_prevSetpoint) / m_setpointTimer.Get();
}





float PIDControl::GetError() const {
 double pidInput;
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  pidInput = m_pidInput->PIDGet();
 }
 return GetSetpoint() - pidInput;
}




void PIDControl::SetPIDSourceType(PIDSourceType pidSource) {
 m_pidInput->SetPIDSourceType(pidSource);
}




PIDSourceType PIDControl::GetPIDSourceType() const {
 return m_pidInput->GetPIDSourceType();
}







float PIDControl::GetAvgError() const {
 float avgError = 0;
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);

  if (m_buf.size())
   avgError = m_bufTotal / m_buf.size();
 }
 return avgError;
}






void PIDControl::SetTolerance(float percent) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
 }
}






void PIDControl::SetPercentTolerance(float percent) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kPercentTolerance;
  m_tolerance = percent;
 }
}






void PIDControl::SetAbsoluteTolerance(float absTolerance) {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_toleranceType = kAbsoluteTolerance;
  m_tolerance = absTolerance;
 }
}
# 472 "./lib/PIDControl/PIDControl.cpp"
void PIDControl::SetToleranceBuffer(unsigned bufLength) {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 m_bufLength = bufLength;


 while (m_buf.size() > bufLength) {
  m_bufTotal -= m_buf.front();
  m_buf.pop();
 }
}
# 493 "./lib/PIDControl/PIDControl.cpp"
bool PIDControl::OnTarget() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 if (m_buf.size() == 0)
  return false;
 double error = GetAvgError();
 switch (m_toleranceType) {
 case kPercentTolerance:
  return fabs(error)
    < m_tolerance / 100 * (m_maximumInput - m_minimumInput);
  break;
 case kAbsoluteTolerance:
  return fabs(error) < m_tolerance;
  break;
 case kNoTolerance:

  return false;
 }
 return false;
}




void PIDControl::Enable() {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_enabled = true;
 }

 if (m_table != nullptr) {
  m_table->PutBoolean("enabled", true);
 }
}




void PIDControl::Disable() {
 {
  std::lock_guard<priority_recursive_mutex> sync(m_mutex);
  m_pidOutput->PIDWrite(0);
  m_enabled = false;
 }

 if (m_table != nullptr) {
  m_table->PutBoolean("enabled", false);
 }
}




bool PIDControl::IsEnabled() const {
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_enabled;
}




void PIDControl::Reset() {
 Disable();

 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 m_prevError = 0;
 m_totalError = 0;
 m_result = 0;
}

void PIDControl::ResetIntegrator()
{
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 m_totalError = 0;
}

double PIDControl::GetIntegratedError() const
{
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 return m_totalError;
}

void PIDControl::SetInvertedOutput(bool inverted)
{
 std::lock_guard<priority_recursive_mutex> sync(m_mutex);
 m_inverted = inverted;

}

std::string PIDControl::GetSmartDashboardType() const {
 return "PIDController";
}

void PIDControl::InitTable(std::shared_ptr<ITable> table) {
 if (m_table != nullptr)
  m_table->RemoveTableListener(this);
 m_table = table;
 if (m_table != nullptr) {
  m_table->PutNumber(kP, GetP());
  m_table->PutNumber(kI, GetI());
  m_table->PutNumber(kD, GetD());
  m_table->PutNumber(kF, GetF());
  m_table->PutNumber(kIntegratedError, GetIntegratedError());
  m_table->PutNumber(kSetpoint, GetSetpoint());
  m_table->PutBoolean(kEnabled, IsEnabled());
  m_table->AddTableListener(this, false);
 }
}

std::shared_ptr<ITable> PIDControl::GetTable() const {
 return m_table;
}

void PIDControl::ValueChanged(ITable* source, llvm::StringRef key,
  std::shared_ptr<nt::Value> value, bool isNew) {
 if (key == kP || key == kI || key == kD || key == kF) {
  if (m_P != m_table->GetNumber(kP, 0.0)
    || m_I != m_table->GetNumber(kI, 0.0)
    || m_D != m_table->GetNumber(kD, 0.0)
    || m_F != m_table->GetNumber(kF, 0.0)) {
   SetPID(m_table->GetNumber(kP, 0.0), m_table->GetNumber(kI, 0.0),
     m_table->GetNumber(kD, 0.0), m_table->GetNumber(kF, 0.0));
  }
 } else if (key == kSetpoint && value->IsDouble()
   && m_setpoint != value->GetDouble()) {
  SetSetpoint(value->GetDouble());
 } else if (key == kEnabled && value->IsBoolean()
   && m_enabled != value->GetBoolean()) {
  if (value->GetBoolean()) {
   Enable();
  } else {
   Disable();
  }
 }
}

void PIDControl::UpdateTable() {
}

void PIDControl::StartLiveWindowMode() {
 Disable();
}

void PIDControl::StopLiveWindowMode() {
}
