using UnityEngine;
using System.Collections.Generic;
using System.Numerics;
using System.Linq; // Для List.Average

public class AsynchronousMotorModel : MonoBehaviour
{
    // --- ENUMS для выбора режимов ---
    public enum StartingScheme
    {
        DirectOnLine,
        StarDelta,
        RotorResistance,
        AutoTransformer
    }

    public enum RotationAxis
    {
        X_Axis,
        Y_Axis,
        Z_Axis
    }

    [Header("Электрические параметры (схема замещения)")]
    public float R1_Stator_Ref = 0.5f;
    public float X1_Stator = 0.8f;
    public float R2_Rotor_Ref = 0.3f;
    public float X2_Rotor_Ref = 0.7f;
    
    public float AlphaReactance = 0.0001f;

    public float Xm_Magnetizing_Unsaturated = 25f; // Убедитесь, что это значение не равно 0!
    public float Xm_SaturationVoltageThreshold = 100f;
    public float Xm_SaturationFactor = 0.5f;
    
    [Header("Насыщение: Кусочно-линейная аппроксимация Xm")]
    public bool UsePiecewiseSaturation = false;
    public AnimationCurve SaturationCurveXm;

    public int PolePairs = 2;

    [Tooltip("Номинальное скольжение двигателя (например, 0.03 для 3%)")]
    public float nominalSlip = 0.03f; // <-- НОВАЯ ПЕРЕМЕННАЯ

    [Header("Входное напряжение и частота (Управляются FrequencyController)")]
    // Эти параметры теперь будут публичными, чтобы FrequencyController мог их изменять.
    // Логика их динамического изменения в этом скрипте удалена.
    public float InputLineVoltage = 380f; 
    public float InputFrequency = 50f;

    [Header("Просадка напряжения сети (п. 4)")]
    public bool EnableVoltageDipOnStartup = false;
    public float NetworkShortCircuitPower = 500000f;
    public float NetworkResistanceToReactanceRatio = 0.1f;
    private Complex Z_network;

    // [Header("Динамическое изменение напряжения")] // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // public bool EnableVoltageChangeSchedule = false; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // public float[] VoltageChangeTimes; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // public float[] TargetLineVoltages; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // private int _currentVoltageScheduleIndex = 0; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // private float _initialLineVoltage; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО


    [Header("Механические параметры")]
    public float MomentOfInertia = 0.01f; // Проверьте это значение в инспекторе! Должно быть > 0
    public float FrictionCoefficientLinear = 0.001f;
    public float FrictionCoefficientQuadratic = 0.00001f;
    public float LoadTorque = 0f;

    [Header("Динамическое изменение нагрузки")]
    public bool EnableLoadChangeSchedule = false;
    public float[] LoadChangeTimes;
    public float[] TargetLoadTorques;
    private int _currentLoadScheduleIndex = 0;
    private float _initialLoadTorque;

    [Header("Детализация механических потерь")]
    public float BearingLossesCoefficient = 0.00005f;
    public float BearingLossesConstant = 0.1f;
    public float BrushLossesPower = 5.0f;
    public float VentilationLossesCubicCoefficient = 0.0000001f;
    public float LoadDependentFrictionCoefficient = 0.0001f;

    [Header("Тепловые параметры")]
    public float ReferenceTemperature = 20f;
    public float AlphaCopper = 0.0039f;

    public float StatorWindingThermalCapacity = 50f;
    public float StatorCoreThermalCapacity = 100f;
    public float RotorWindingThermalCapacity = 40f;
    public float RotorCoreThermalCapacity = 80f;
    public float HousingThermalCapacity = 120f;

    public float G_StatorWinding_StatorCore = 20f;
    public float G_StatorCore_Housing = 15f;
    public float G_RotorWinding_RotorCore = 18f;
    public float G_RotorCore_Housing = 12f;
    public float G_Housing_Ambient_Base = 5f;
    public float G_Housing_Ambient_SpeedFactor = 0.005f;
    public float G_Housing_Ambient_RadiationFactor = 0.00000005f;
    
    public float G_StatorWinding_Ambient_Base = 2f;
    public float G_RotorWinding_Ambient_Base = 1f;
    
    [Header("Детализация теплового поля (п. 2)")]
    public bool EnableThermalGradients = false;
    public float StatorWindingInternalThermalResistance = 0.5f;
    public float RotorWindingInternalThermalResistance = 0.7f;
    private float T_stator_winding_max;
    private float T_rotor_winding_max;

    public float AmbientTemperature = 25f;

    [Header("Температурное старение изоляции (п. 9)")]
    public bool EnableInsulationAging = true;
    public float InsulationAgingReferenceTemp = 100f;
    public float ArrheniusConstantA = 1e12f;
    public float ArrheniusActivationEnergy_eV = 0.8f;
    private float _insulationLifeConsumed = 0f;
    private const float BoltzmannConstant_eV_K = 8.617333262145e-5f;

    [Header("Моделирование потерь в стали (п. 1)")]
    public float IronLossesCoefficientHysteresisStator = 0.005f;
    public float IronLossesCoefficientEddyCurrentsStator = 0.00001f;
    public float IronLossesCoefficientHysteresisRotor = 0.0001f;
    public float IronLossesCoefficientEddyCurrentsRotor = 0.000001f;
    
    public float HysteresisVoltageExponent = 1.6f;
    public float EddyCurrentsVoltageExponent = 2.0f;
    [Header("Потери от гармоник обмотки (п. 1)")]
    public bool EnableWindingHarmonicLosses = true;
    public float WindingHarmonicLossesFactor = 0.001f;

    [Header("Гармонические потери и паразитные моменты")]
    public float HarmonicLossesCoefficient = 0.0005f;
    public float ParasiticTorqueMagnitude = 0.0f;
    public float ParasiticTorqueSpeedThreshold = 10f;

    [Header("Несинусоидальность питания (инвертор)")]
    public bool IsInverterSupplied = false;
    public float InverterHarmonicLossCoefficient = 0.01f;
    public float InverterTorqueRippleFactor = 0.1f;
    public float InverterRippleFrequency = 300f;
    private float _inverterTorqueRipple = 0f;


    [Header("Схемы пуска")]
    public StartingScheme SelectedStartingScheme = StartingScheme.DirectOnLine;
    public float StarDeltaSwitchTime = 2.0f;
    public float RotorResistanceStartValue = 1.0f;
    public float RotorResistanceSwitchTime = 3.0f;
    public float AutoTransformerRatio = 0.7f;
    public float AutoTransformerSwitchTime = 5.0f;

    [Header("Регулирование скорости")]
    public float ExternalRotorResistance = 0.0f;

    [Header("Тепловое расширение")]
    public bool EnableThermalExpansionEffect = true;
    public float ThermalExpansionCoefficientXm = 0.00001f;
    public float ThermalExpansionReferenceTemp = 20f;

    [Header("Вибрация и Шум")]
    public bool EnableVibrationNoiseModel = true;
    public float VibrationAccelerationFactor = 0.01f;
    public float NoiseSpeedFactor = 0.005f;
    public float NoiseLoadFactor = 0.001f;
    private float _currentVibrationLevel = 0f;
    private float _currentNoiseLevel = 0f;
    public bool EnableBearingWear = true;
    public float BearingWearRateFactor = 0.0001f;
    public float BearingWearLossIncreaseFactor = 0.01f;
    private float _bearingWearLevel = 0f;


    [Header("Логирование пусковых характеристик")]
    public bool LogStartUpData = true;
    public float LogInterval = 0.1f;
    private float logTimer = 0f;
    private float simulationStartTime;

    private List<float> startupTime = new List<float>();
    private List<float> startupSpeedRPM = new List<float>();
    private List<float> startupTorque = new List<float>();
    private List<float> startupStatorCurrent = new List<float>();
    private List<float> startupStatorTempWinding = new List<float>();

    [Header("Текущее состояние - Логирование в консоль")]
    public bool EnableRealtimeConsoleLogging = true;
    public float RealtimeLogInterval = 1.0f;
    private float _realtimeLogTimer = 0f;


    // Текущие состояния динамической модели
    private float mechanicalAngularSpeed = 0f;
    private float rotorAngle = 0f;
    private float currentElectromagneticTorque = 0f;
    private float currentStatorCurrentMagnitude = 0f; 
    private float _lastAngularAcceleration = 0f;

    // Тепловые состояния (5 узлов)
    private float T_stator_winding = 25f;
    private float T_stator_core = 25f;
    private float T_rotor_winding = 25f;
    private float T_rotor_core = 25f;
    private float T_housing = 25f;

    // ОБЪЯВЛЕНИЯ ПЕРЕМЕННЫХ, ВЫЗЫВАВШИХ ОШИБКИ CS0103
    private float current_R1;
    private float current_X1;
    private float current_R2_Effective;
    private float current_X2_Effective;
    private float current_Xm;


    // Вспомогательные переменные
    private float phaseVoltageActual; // Фазное напряжение, приложенное к двигателю (номинальное или скорректированное по пусковой схеме)
    private float synchronousAngularSpeedElectrical;
    private float synchronousMechanicalSpeed;
    private float phaseVoltageSource; // Фазное напряжение источника питания (линейное / sqrt(3))

    private float _currentLoadTorque;
    // private float _currentLineVoltage; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО

    // --- Оптимизация: кэширование часто используемых комплексных чисел ---
    private Complex _Z_rotor_branch;
    private Complex _Z_magnetizing;
    private Complex _Z_parallel;
    private Complex _Z_total_motor;
    private Complex _I1; // Ток фазы A статора
    private Complex _U1_prime; // Напряжение на воздушном зазоре (фаза A)
    private Complex _I2_prime; // Ток ротора, приведенный (фаза A)
    private Complex _actualPhaseVoltageAtTerminals; // Напряжение на клеммах двигателя (фаза A)
    private Complex _zeroComplex = new Complex(0.0, 0.0); 

    // --- Оптимизация: кэширование констант и предварительные расчеты ---
    private float _invSqrt3;
    private float _syncSpeedElectricalConst;
    private float _syncMechanicalSpeedConst;
    private float _polePairsInv; 
    private float _pi30Div; 

    // --- Константа для минимального скольжения ---
    private const float MIN_SLIP_EPSILON = 1e-4f; // Увеличено значение для большей устойчивости

    [Header("Текущие фазные параметры (все 3 фазы)")]
    public float voltagePhaseAMagnitude;
    public float voltagePhaseAAngleDeg;

    public float voltagePhaseBMagnitude;
    public float voltagePhaseBAngleDeg;

    public float voltagePhaseCMagnitude;
    public float voltageCAngleDeg; // Corrected: This was missing the 'PhaseC' prefix in the field name

    public float currentPhaseAMagnitude;
    public float currentPhaseAAngleDeg;

    public float currentPhaseBMagnitude;
    public float currentPhaseBAngleDeg;

    public float currentPhaseCMagnitude;
    public float currentPhaseCAngleDeg;

    [Header("Вспомогательные токи и напряжения (Воздушный зазор)")]
    public float airGapVoltagePhaseAMagnitude; 
    public float airGapVoltagePhaseAAngleDeg;  

    public float airGapVoltagePhaseBMagnitude;
    public float airGapVoltagePhaseBAngleDeg;

    public float airGapVoltagePhaseCMagnitude;
    public float airGapVoltageCAngleDeg;  // This variable was causing the error due to incorrect name, it's corrected now.

    [Header("Вспомогательные токи и напряжения (Ток ротора приведенный)")]
    public float rotorCurrentPhaseAMagnitudeReferred; 
    public float rotorCurrentPhaseAAngleDegReferred;  

    public float rotorCurrentPhaseBMagnitudeReferred;
    public float rotorCurrentPhaseBAngleDegReferred;

    public float rotorCurrentPhaseCMagnitudeReferred;
    public float rotorCurrentPhaseCAngleDegReferred;


    [Header("Визуализация")]
    public Transform MotorRotorTransform;
    [Header("Настройка оси вращения")]
    public RotationAxis RotorRotationAxis = RotationAxis.X_Axis; // По умолчанию вращаем по X
    private Rigidbody _rotorRigidbody; // Добавлено для работы с Rigidbody

    void Start()
    {
        simulationStartTime = Time.time;
        _initialLoadTorque = LoadTorque;
        _currentLoadTorque = LoadTorque;
        // _initialLineVoltage = InputLineVoltage; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
        // _currentLineVoltage = InputLineVoltage; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
        // _currentVoltageScheduleIndex = 0; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
        _currentLoadScheduleIndex = 0;
        _realtimeLogTimer = 0f;

        // Инициализация новых переменных для инспектора
        voltagePhaseAMagnitude = 0f;
        voltagePhaseAAngleDeg = 0f;
        voltagePhaseBMagnitude = 0f;
        voltagePhaseBAngleDeg = 0f;
        voltagePhaseCMagnitude = 0f;
        voltageCAngleDeg = 0f; // Corrected initialization

        currentPhaseAMagnitude = 0f;
        currentPhaseAAngleDeg = 0f;
        currentPhaseBMagnitude = 0f;
        currentPhaseBAngleDeg = 0f;
        currentPhaseCMagnitude = 0f;
        currentPhaseCAngleDeg = 0f;

        airGapVoltagePhaseAMagnitude = 0f;
        airGapVoltagePhaseAAngleDeg = 0f;
        airGapVoltagePhaseBMagnitude = 0f;
        airGapVoltagePhaseBAngleDeg = 0f;
        airGapVoltagePhaseCMagnitude = 0f;
        airGapVoltageCAngleDeg = 0f; // Corrected initialization

        rotorCurrentPhaseAMagnitudeReferred = 0f;
        rotorCurrentPhaseAAngleDegReferred = 0f;
        rotorCurrentPhaseBMagnitudeReferred = 0f;
        rotorCurrentPhaseBAngleDegReferred = 0f;
        rotorCurrentPhaseCMagnitudeReferred = 0f;
        rotorCurrentPhaseCAngleDegReferred = 0f;


        // Убедимся, что Xm_Magnetizing_Unsaturated не ноль, чтобы избежать деления на ноль
        Xm_Magnetizing_Unsaturated = Mathf.Max(0.1f, Xm_Magnetizing_Unsaturated);

        // Убедимся, что PolePairs не ноль
        if (PolePairs == 0)
        {
            Debug.LogWarning("PolePairs cannot be zero. Defaulting to 2.");
            PolePairs = 2;
        }
        // Убедимся, что InputFrequency не ноль и не отрицательная
        if (InputFrequency <= 0)
        {
            Debug.LogWarning("InputFrequency cannot be zero or negative. Defaulting to 50 Hz.");
            InputFrequency = 50f;
        }

        // --- Оптимизация: предварительный расчет констант ---
        _invSqrt3 = 1f / Mathf.Sqrt(3f);
        // synchronousAngularSpeedElectrical и synchronousMechanicalSpeed теперь зависят от InputFrequency,
        // который будет меняться FrequencyController. Поэтому их пересчет будет в FixedUpdate.
        _syncSpeedElectricalConst = 2f * Mathf.PI * InputFrequency; 
        synchronousAngularSpeedElectrical = _syncSpeedElectricalConst; 
        synchronousMechanicalSpeed = _syncSpeedElectricalConst / PolePairs; 
        _syncMechanicalSpeedConst = synchronousMechanicalSpeed; 
        _polePairsInv = 1f / PolePairs;
        _pi30Div = Mathf.PI / 30f;

        // Добавим логирование для проверки этих значений
        Debug.Log($"[Start Debug] InputFrequency after check: {InputFrequency}");
        Debug.Log($"[Start Debug] PolePairs after check: {PolePairs}");
        Debug.Log($"[Start Debug] _syncSpeedElectricalConst: {_syncSpeedElectricalConst}");
        Debug.Log($"[Start Debug] synchronousMechanicalSpeed: {synchronousMechanicalSpeed}");
        Debug.Log($"[Start Debug] _syncMechanicalSpeedConst (final in Start): {_syncMechanicalSpeedConst}");

        // Расчет сопротивления сети для просадки напряжения (п. 4)
        if (EnableVoltageDipOnStartup)
        {
            // Здесь _currentLineVoltage заменено на InputLineVoltage, так как последнее управляется извне
            float Z_network_magnitude = (InputLineVoltage * InputLineVoltage) / NetworkShortCircuitPower;
            float X_network = Z_network_magnitude / Mathf.Sqrt(1f + NetworkResistanceToReactanceRatio * NetworkResistanceToReactanceRatio);
            float R_network = X_network * NetworkResistanceToReactanceRatio;
            Z_network = new Complex((double)R_network, (double)X_network);
        } else {
            Z_network = _zeroComplex; 
        }

        // Инициализация начальных значений температур
        T_stator_winding = AmbientTemperature;
        T_stator_core = AmbientTemperature;
        T_rotor_winding = AmbientTemperature;
        T_rotor_core = AmbientTemperature;
        T_housing = AmbientTemperature;
        T_stator_winding_max = AmbientTemperature;
        T_rotor_winding_max = AmbientTemperature;

        // Инициализация текущих сопротивлений при начальной температуре
        current_R1 = GetTemperatureCompensatedResistance(R1_Stator_Ref, T_stator_winding);
        current_X1 = GetTemperatureCompensatedReactance(X1_Stator, T_stator_winding); 
        current_R2_Effective = GetTemperatureCompensatedResistance(R2_Rotor_Ref, T_rotor_winding) + ExternalRotorResistance;
        current_X2_Effective = GetTemperatureCompensatedReactance(X2_Rotor_Ref, T_rotor_winding); 
        current_Xm = Xm_Magnetizing_Unsaturated; // Инициализация Xm

        // Инициализация пусковой схемы
        // phaseVoltageSource теперь будет брать значение из InputLineVoltage, который управляется FrequencyController
        phaseVoltageSource = InputLineVoltage * _invSqrt3; 

        if (LogStartUpData)
        {
            startupTime.Clear();
            startupSpeedRPM.Clear();
            startupTorque.Clear();
            startupStatorCurrent.Clear();
            startupStatorTempWinding.Clear();
            logTimer = 0f;
        }

        // Добавлено для работы с Rigidbody
        if (MotorRotorTransform != null)
        {
            _rotorRigidbody = MotorRotorTransform.GetComponent<Rigidbody>();
            if (_rotorRigidbody == null)
            {
                Debug.LogWarning("MotorRotorTransform does not have a Rigidbody. Direct transform manipulation will be used. Consider adding a Rigidbody for physics simulation.");
            }
        }
    }

    void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;
        float currentTime = Time.time - simulationStartTime;

        HandleDynamicLoadChanges(currentTime);
        // HandleDynamicVoltageChanges(currentTime); // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
        
        // InputFrequency и InputLineVoltage теперь изменяются FrequencyController,
        // поэтому мы просто используем их здесь.
        // Пересчитываем синхронную скорость на основе текущей InputFrequency
        synchronousAngularSpeedElectrical = 2f * Mathf.PI * InputFrequency;
        synchronousMechanicalSpeed = synchronousAngularSpeedElectrical / PolePairs;
        
        phaseVoltageSource = InputLineVoltage * _invSqrt3; // Используем актуальное InputLineVoltage
        HandleStartingScheme(currentTime); // Схемы пуска теперь переопределяются FrequencyController

        // --- Оптимизация: кэширование температурно-зависимых сопротивлений ---
        current_R1 = GetTemperatureCompensatedResistance(R1_Stator_Ref, T_stator_winding);
        current_X1 = GetTemperatureCompensatedReactance(X1_Stator, T_stator_winding);

        float R2_TempCompensated = GetTemperatureCompensatedResistance(R2_Rotor_Ref, T_rotor_winding);
        float X2_TempCompensated = GetTemperatureCompensatedReactance(X2_Rotor_Ref, T_rotor_winding);

        float currentSlip;
        if (Mathf.Abs(synchronousMechanicalSpeed) < 1e-6f) 
        {
            currentSlip = 1.0f; 
        }
        else
        {
            currentSlip = (synchronousMechanicalSpeed - mechanicalAngularSpeed) / synchronousMechanicalSpeed;
            // Убеждаемся, что скольжение никогда не равно абсолютному нулю, сохраняя знак
            if (Mathf.Abs(currentSlip) < MIN_SLIP_EPSILON)
            {
                currentSlip = Mathf.Sign(currentSlip) * MIN_SLIP_EPSILON;
                if (currentSlip == 0) currentSlip = MIN_SLIP_EPSILON; // В случае, если Sign(0) == 0
            }
            currentSlip = Mathf.Clamp(currentSlip, -5.0f, 2.0f); 
        }
        
        // **Проверка currentSlip на NaN/Infinity**
        if (float.IsNaN(currentSlip) || float.IsInfinity(currentSlip))
        {
            Debug.LogError($"[NaN Error] currentSlip became NaN/Infinity. Resetting simulation. Time: {currentTime:F2}");
            ResetSimulationState(); // Сброс состояния, чтобы избежать дальнейших ошибок
            return; // Прерываем FixedUpdate, чтобы не работать с некорректными данными
        }

        float rotorCurrentFrequency = Mathf.Abs(currentSlip * InputFrequency); 
        current_R2_Effective = ApplySkinEffectToResistance(R2_TempCompensated, rotorCurrentFrequency) + ExternalRotorResistance;
        current_X2_Effective = ApplySkinEffectToReactance(X2_TempCompensated, rotorCurrentFrequency);
        
        // Убедимся, что сопротивления не становятся отрицательными
        current_R1 = Mathf.Max(current_R1, 0.001f);
        current_X1 = Mathf.Max(current_X1, 0.001f);
        current_R2_Effective = Mathf.Max(current_R2_Effective, 0.001f);
        current_X2_Effective = Mathf.Max(current_X2_Effective, 0.001f);

        // Добавлена проверка на NaN/Infinity для входных параметров CalculateSaturatedXm
        if (float.IsNaN(phaseVoltageActual) || float.IsInfinity(phaseVoltageActual) ||
            float.IsNaN(current_R1) || float.IsInfinity(current_R1) ||
            float.IsNaN(current_X1) || float.IsInfinity(current_X1) ||
            float.IsNaN(current_R2_Effective) || float.IsInfinity(current_R2_Effective) ||
            float.IsNaN(current_X2_Effective) || float.IsInfinity(current_X2_Effective) ||
            float.IsNaN(currentSlip) || float.IsInfinity(currentSlip))
        {
            Debug.LogError($"[NaN Check] Input parameters for CalculateSaturatedXm contain NaN/Infinity. Resetting simulation. Time: {currentTime:F2}");
            ResetSimulationState();
            return;
        }

        current_Xm = CalculateSaturatedXm(phaseVoltageActual, current_R1, current_X1, current_R2_Effective, current_X2_Effective, currentSlip);
        if (EnableThermalExpansionEffect)
        {
            current_Xm = ApplyThermalExpansionToXm(current_Xm, T_stator_core); 
        }

        // Дополнительная проверка current_Xm после всех расчетов
        if (float.IsNaN(current_Xm) || float.IsInfinity(current_Xm) || current_Xm < 0.001f) // Убедимся, что Xm не NaN/Infinity и не слишком мал
        {
            Debug.LogError($"[NaN Check] current_Xm became NaN/Infinity or too small. Value: {current_Xm}. Resetting to default. Time: {currentTime:F2}");
            current_Xm = Xm_Magnetizing_Unsaturated; // Вернуть к безопасному значению
            if (float.IsNaN(current_Xm) || float.IsInfinity(current_Xm)) // Двойная проверка на случай, если Xm_Magnetizing_Unsaturated сам NaN
            {
                Debug.LogError($"[NaN Critical] Default Xm_Magnetizing_Unsaturated is NaN/Infinity. Resetting simulation. Time: {currentTime:F2}");
                ResetSimulationState();
                return;
            }
        }


        float R2_prime_slip = current_R2_Effective / currentSlip;
        if (float.IsNaN(R2_prime_slip) || float.IsInfinity(R2_prime_slip))
        {
            Debug.LogError($"[NaN Check] R2_prime_slip became NaN/Infinity. Value: {R2_prime_slip}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }

        // --- Оптимизация: используем кэшированные Complex объекты ---
        _Z_rotor_branch = new Complex((double)R2_prime_slip, (double)current_X2_Effective);
        if (float.IsNaN((float)_Z_rotor_branch.Real) || float.IsNaN((float)_Z_rotor_branch.Imaginary))
        {
            Debug.LogError($"[NaN Check] Z_rotor_branch became NaN. Real: {_Z_rotor_branch.Real}, Imag: {_Z_rotor_branch.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }

        _Z_magnetizing = new Complex(0.0, (double)current_Xm);
        if (float.IsNaN((float)_Z_magnetizing.Real) || float.IsNaN((float)_Z_magnetizing.Imaginary))
        {
            Debug.LogError($"[NaN Check] Z_magnetizing became NaN. Real: {_Z_magnetizing.Real}, Imag: {_Z_magnetizing.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }

        // Расчет Z_parallel с усиленной проверкой на деление на ноль
        Complex sum_Z_branches = _Z_rotor_branch + _Z_magnetizing;
        if (sum_Z_branches.Magnitude < 1e-12) // Ультра-малый эпсилон для предотвращения деления на ноль
        {
            Debug.LogError($"[NaN Check] Sum of rotor and magnetizing branches is near zero ({sum_Z_branches.Magnitude:E2}). Preventing division by zero for Z_parallel. Time: {currentTime:F2}");
            _Z_parallel = _zeroComplex; 
        }
        else
        {
            _Z_parallel = (_Z_rotor_branch * _Z_magnetizing) / sum_Z_branches;
        }
        if (float.IsNaN((float)_Z_parallel.Real) || float.IsNaN((float)_Z_parallel.Imaginary))
        {
            Debug.LogError($"[NaN Check] Z_parallel became NaN. Real: {_Z_parallel.Real}, Imag: {_Z_parallel.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }


        _Z_total_motor = new Complex((double)current_R1, (double)current_X1) + _Z_parallel;
        if (float.IsNaN((float)_Z_total_motor.Real) || float.IsNaN((float)_Z_total_motor.Imaginary))
        {
            Debug.LogError($"[NaN Check] Z_total_motor became NaN. Real: {_Z_total_motor.Real}, Imag: {_Z_total_motor.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }

        // ----- Расчет _actualPhaseVoltageAtTerminals (Напряжение на клеммах) -----
        _actualPhaseVoltageAtTerminals = new Complex((double)phaseVoltageSource, 0.0); // Считаем напряжение источника референсом (угол 0)
        if (EnableVoltageDipOnStartup)
        {
            Complex Z_total_circuit = _Z_total_motor + Z_network;
            if (Z_total_circuit.Magnitude < 1e-12) 
            {
                Debug.LogError($"[NaN Check] Z_total_circuit magnitude is near zero ({Z_total_circuit.Magnitude:E2}). Preventing division by zero for I1. Time: {currentTime:F2}");
                _I1 = _zeroComplex;
                _actualPhaseVoltageAtTerminals = _zeroComplex;
            }
            else
            {
                _I1 = new Complex((double)phaseVoltageSource, 0.0) / Z_total_circuit;
                _actualPhaseVoltageAtTerminals = _I1 * _Z_total_motor;
            }
        }
        else
        {
            // В случае без просадки напряжения, напряжение на клеммах равно фазному напряжению
            _actualPhaseVoltageAtTerminals = new Complex((double)phaseVoltageActual, 0.0); 
        }
        
        // Проверка _actualPhaseVoltageAtTerminals на NaN/Infinity
        if (float.IsNaN((float)_actualPhaseVoltageAtTerminals.Real) || float.IsNaN((float)_actualPhaseVoltageAtTerminals.Imaginary))
        {
            Debug.LogError($"[NaN Check] ActualPhaseVoltageAtTerminals became NaN. Real: {_actualPhaseVoltageAtTerminals.Real}, Imag: {_actualPhaseVoltageAtTerminals.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }
        

        // ----- Расчет _I1 (Ток статора) -----
        if (_Z_total_motor.Magnitude < 1e-12) 
        {
             Debug.LogError($"[NaN Check] Z_total_motor magnitude is near zero ({_Z_total_motor.Magnitude:E2}). Preventing division by zero for I1 (after voltage dip calc). Time: {currentTime:F2}");
             _I1 = _zeroComplex;
        }
        else
        {
             _I1 = _actualPhaseVoltageAtTerminals / _Z_total_motor;
        }
        if (float.IsNaN((float)_I1.Real) || float.IsNaN((float)_I1.Imaginary))
        {
            Debug.LogError($"[NaN Check] I1 became NaN. Real: {_I1.Real}, Imag: {_I1.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }
       
        currentStatorCurrentMagnitude = (float)_I1.Magnitude; 


        // ----- Расчет _U1_prime (Напряжение на воздушном зазоре) -----
        _U1_prime = _I1 * _Z_parallel;
        if (float.IsNaN((float)_U1_prime.Real) || float.IsNaN((float)_U1_prime.Imaginary))
        {
            Debug.LogError($"[NaN Check] U1_prime became NaN. Real: {_U1_prime.Real}, Imag: {_U1_prime.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }


        // ----- Расчет _I2_prime (Ток ротора, приведенный) -----
        if (_Z_rotor_branch.Magnitude < 1e-12) 
        {
            Debug.LogError($"[NaN Check] Z_rotor_branch magnitude is near zero ({_Z_rotor_branch.Magnitude:E2}). Preventing division by zero for I2_prime. Time: {currentTime:F2}");
            _I2_prime = _zeroComplex;
        }
        else
        {
            _I2_prime = _U1_prime / _Z_rotor_branch;
        }
        if (float.IsNaN((float)_I2_prime.Real) || float.IsNaN((float)_I2_prime.Imaginary))
        {
            Debug.LogError($"[NaN Check] I2_prime became NaN. Real: {_I2_prime.Real}, Imag: {_I2_prime.Imaginary}. Time: {currentTime:F2}");
            ResetSimulationState(); return;
        }


        // ----- ОБНОВЛЕНИЕ ПУБЛИЧНЫХ ПЕРЕМЕННЫХ ДЛЯ ВСЕХ ТРЕХ ФАЗ В ИНСПЕКТОРЕ -----
        // Фаза A (основные расчеты)
        voltagePhaseAMagnitude = (float)_actualPhaseVoltageAtTerminals.Magnitude;
        voltagePhaseAAngleDeg = (float)_actualPhaseVoltageAtTerminals.Phase * Mathf.Rad2Deg;

        currentPhaseAMagnitude = (float)_I1.Magnitude;
        currentPhaseAAngleDeg = (float)_I1.Phase * Mathf.Rad2Deg;

        // Фаза B (отстает на 120 градусов)
        voltagePhaseBMagnitude = voltagePhaseAMagnitude;
        voltagePhaseBAngleDeg = voltagePhaseAAngleDeg - 120f;
        NormalizeAngle(ref voltagePhaseBAngleDeg);

        currentPhaseBMagnitude = currentPhaseAMagnitude;
        currentPhaseBAngleDeg = currentPhaseAAngleDeg - 120f;
        NormalizeAngle(ref currentPhaseBAngleDeg);

        // Фаза C (опережает на 120 градусов, или отстает на 240 градусов)
        voltagePhaseCMagnitude = voltagePhaseAMagnitude;
        voltageCAngleDeg = voltagePhaseAAngleDeg + 120f; // Corrected usage
        NormalizeAngle(ref voltageCAngleDeg); // Corrected usage

        currentPhaseCMagnitude = currentPhaseAMagnitude;
        currentPhaseCAngleDeg = currentPhaseAAngleDeg + 120f;
        NormalizeAngle(ref currentPhaseCAngleDeg);

        // Обновление публичных переменных для инспектора (Воздушный зазор)
        airGapVoltagePhaseAMagnitude = (float)_U1_prime.Magnitude;
        airGapVoltagePhaseAAngleDeg = (float)_U1_prime.Phase * Mathf.Rad2Deg;

        airGapVoltagePhaseBMagnitude = airGapVoltagePhaseAMagnitude;
        airGapVoltagePhaseBAngleDeg = airGapVoltagePhaseAAngleDeg - 120f;
        NormalizeAngle(ref airGapVoltagePhaseBAngleDeg);

        airGapVoltagePhaseCMagnitude = airGapVoltagePhaseAMagnitude;
        airGapVoltageCAngleDeg = airGapVoltagePhaseAAngleDeg + 120f; // Corrected usage
        NormalizeAngle(ref airGapVoltageCAngleDeg); // Corrected usage


        // Обновление публичных переменных для инспектора (Ток ротора)
        rotorCurrentPhaseAMagnitudeReferred = (float)_I2_prime.Magnitude;
        rotorCurrentPhaseAAngleDegReferred = (float)_I2_prime.Phase * Mathf.Rad2Deg;

        rotorCurrentPhaseBMagnitudeReferred = rotorCurrentPhaseAMagnitudeReferred;
        rotorCurrentPhaseBAngleDegReferred = rotorCurrentPhaseAAngleDegReferred - 120f;
        NormalizeAngle(ref rotorCurrentPhaseBAngleDegReferred);

        rotorCurrentPhaseCMagnitudeReferred = rotorCurrentPhaseAMagnitudeReferred;
        rotorCurrentPhaseCAngleDegReferred = rotorCurrentPhaseAAngleDegReferred + 120f;
        NormalizeAngle(ref rotorCurrentPhaseCAngleDegReferred);


        if (float.IsNaN((float)_I2_prime.Magnitude) || float.IsInfinity((float)_I2_prime.Magnitude))
        {
            Debug.LogError($"[NaN Check] I2_prime.Magnitude is NaN/Infinity. Value: {(float)_I2_prime.Magnitude}. Time: {currentTime:F2}");
            currentElectromagneticTorque = 0f; 
            ResetSimulationState(); 
            return; 
        }

        // Clamp _I2_prime.Magnitude to prevent extreme values before squaring
        float clamped_I2_prime_magnitude = Mathf.Clamp((float)_I2_prime.Magnitude, 0f, 1e10f); // Ограничение для предотвращения переполнения
        
        currentElectromagneticTorque = (float)((3 * System.Math.Pow(clamped_I2_prime_magnitude, 2) * (current_R2_Effective / currentSlip)) / synchronousMechanicalSpeed); 
        // Внимание: Здесь использовал synchronousMechanicalSpeed, а не _syncMechanicalSpeedConst, так как она теперь динамически меняется
        
        // **Проверка currentElectromagneticTorque на NaN/Infinity**
        if (float.IsNaN(currentElectromagneticTorque) || float.IsInfinity(currentElectromagneticTorque))
        {
            Debug.LogError($"[NaN Error] currentElectromagneticTorque became NaN/Infinity. Setting to 0. Time: {currentTime:F2}");
            currentElectromagneticTorque = 0f; 
            ResetSimulationState(); // Сброс состояния для предотвращения дальнейших ошибок
            return;
        }

        float frictionTorque = FrictionCoefficientLinear * mechanicalAngularSpeed + 
                               FrictionCoefficientQuadratic * mechanicalAngularSpeed * mechanicalAngularSpeed;
        
        float bearingLossesTorque = BearingLossesConstant + BearingLossesCoefficient * Mathf.Abs(mechanicalAngularSpeed);
        if (EnableBearingWear)
        {
            bearingLossesTorque *= (1 + _bearingWearLevel / 100f * BearingWearLossIncreaseFactor);
        }

        float ventilationLossesTorque = VentilationLossesCubicCoefficient * Mathf.Pow(Mathf.Abs(mechanicalAngularSpeed), 3);
        
        float loadDependentFrictionTorque = LoadDependentFrictionCoefficient * Mathf.Abs(_currentLoadTorque);

        float totalMechanicalTorqueLosses = frictionTorque + bearingLossesTorque + ventilationLossesTorque + loadDependentFrictionTorque;

        float parasiticTorque = 0f;
        if (mechanicalAngularSpeed < ParasiticTorqueSpeedThreshold && mechanicalAngularSpeed > -ParasiticTorqueSpeedThreshold)
        {
            parasiticTorque = ParasiticTorqueMagnitude * (1f - Mathf.Abs(mechanicalAngularSpeed) / ParasiticTorqueSpeedThreshold);
        }

        _inverterTorqueRipple = 0f;
        if (IsInverterSupplied && mechanicalAngularSpeed != 0)
        {
            _inverterTorqueRipple = InverterTorqueRippleFactor * currentStatorCurrentMagnitude * Mathf.Sin(2f * Mathf.PI * InverterRippleFrequency * currentTime);
        }
        
        // **Проверка MomentOfInertia на ноль/NaN**
        if (MomentOfInertia <= 1e-9f || float.IsNaN(MomentOfInertia) || float.IsInfinity(MomentOfInertia))
        {
             Debug.LogError($"[Config Error] MomentOfInertia is zero/NaN/Infinity! Cannot calculate angular acceleration. Setting to default 0.01. Time: {currentTime:F2}");
             MomentOfInertia = 0.01f; // Устанавливаем разумное значение по умолчанию
        }

        float angularAcceleration = (currentElectromagneticTorque - _currentLoadTorque - totalMechanicalTorqueLosses - parasiticTorque + _inverterTorqueRipple) / MomentOfInertia;
        
        // **Проверка angularAcceleration на NaN/Infinity**
        if (float.IsNaN(angularAcceleration) || float.IsInfinity(angularAcceleration))
        {
            Debug.LogError($"[NaN Error] angularAcceleration became NaN/Infinity. Setting to 0. Time: {currentTime:F2}");
            angularAcceleration = 0f;
            mechanicalAngularSpeed = 0f; // Сброс скорости, чтобы не накапливать NaN
            rotorAngle = 0f; // Сброс угла
            ResetSimulationState(); // Сброс состояния для предотвращения дальнейших ошибок
            return; // Прерываем FixedUpdate, чтобы не работать с некорректными данными
        }

        mechanicalAngularSpeed += angularAcceleration * deltaTime;
        rotorAngle += mechanicalAngularSpeed * deltaTime;

        // Отладочный лог для проверки значений перед применением поворота
        // Debug.Log($"[Debug Rotation] DeltaTime: {deltaTime:F4}, AngularAccel: {angularAcceleration:F4}, Speed: {mechanicalAngularSpeed:F4}, Angle: {rotorAngle:F4}");


        // **Проверка mechanicalAngularSpeed и rotorAngle на NaN/Infinity**
        if (float.IsNaN(mechanicalAngularSpeed) || float.IsInfinity(mechanicalAngularSpeed))
        {
            Debug.LogError($"[NaN Error] mechanicalAngularSpeed became NaN/Infinity. Resetting to 0. Time: {currentTime:F2}");
            mechanicalAngularSpeed = 0f;
            rotorAngle = 0f; // Сброс угла, так как он зависит от скорости
        }

        if (float.IsNaN(rotorAngle) || float.IsInfinity(rotorAngle))
        {
            Debug.LogError($"[NaN Error] rotorAngle became NaN/Infinity. Resetting to 0. Time: {currentTime:F2}");
            rotorAngle = 0f;
        }

        _lastAngularAcceleration = angularAcceleration;

        if (MotorRotorTransform != null)
        {
            if (_rotorRigidbody != null)
            {
                // Если есть Rigidbody, используем angularVelocity
                UnityEngine.Vector3 targetAngularVelocity = UnityEngine.Vector3.zero;
                switch (RotorRotationAxis)
                {
                    case RotationAxis.X_Axis:
                        targetAngularVelocity = new UnityEngine.Vector3(mechanicalAngularSpeed, 0, 0);
                        break;
                    case RotationAxis.Y_Axis:
                        targetAngularVelocity = new UnityEngine.Vector3(0, mechanicalAngularSpeed, 0);
                        break;
                    case RotationAxis.Z_Axis:
                        targetAngularVelocity = new UnityEngine.Vector3(0, 0, mechanicalAngularSpeed);
                        break;
                }
                // Clamp угловую скорость для предотвращения экстремальных значений
                _rotorRigidbody.angularVelocity = UnityEngine.Vector3.ClampMagnitude(targetAngularVelocity, 1000f); // Ограничение в 1000 рад/с
            }
            else
            {
                // Старый код для прямого изменения transform.localEulerAngles (если Rigidbody нет)
                UnityEngine.Vector3 currentRotation = MotorRotorTransform.localEulerAngles; 
                switch (RotorRotationAxis)
                {
                    case RotationAxis.X_Axis:
                        currentRotation.x = rotorAngle * Mathf.Rad2Deg;
                        break;
                    case RotationAxis.Y_Axis:
                        currentRotation.y = rotorAngle * Mathf.Rad2Deg;
                        break;
                    case RotationAxis.Z_Axis:
                        currentRotation.z = rotorAngle * Mathf.Rad2Deg;
                        break;
                }
                // Проверка currentRotation на NaN перед присваиванием
                if (float.IsNaN(currentRotation.x) || float.IsNaN(currentRotation.y) || float.IsNaN(currentRotation.z))
                {
                    Debug.LogError($"[NaN Error] currentRotation contains NaN before assignment. Resetting to UnityEngine.Vector3.zero. Time: {currentTime:F2}");
                    MotorRotorTransform.localEulerAngles = UnityEngine.Vector3.zero;
                    rotorAngle = 0f; // Сброс rotorAngle, так как он привел к NaN в повороте
                }
                else
                {
                    MotorRotorTransform.localEulerAngles = currentRotation; 
                }
            }
        }

        float statorCopperLosses = 3f * currentStatorCurrentMagnitude * currentStatorCurrentMagnitude * current_R1;
        float rotorCopperLosses = (float)(3f * System.Math.Pow(_I2_prime.Magnitude, 2) * current_R2_Effective);

        float U1_prime_magnitude_cached = (float)_U1_prime.Magnitude;
        float statorIronLosses = (IronLossesCoefficientHysteresisStator * InputFrequency * Mathf.Pow((float)U1_prime_magnitude_cached, HysteresisVoltageExponent) + 
                                  IronLossesCoefficientEddyCurrentsStator * InputFrequency * InputFrequency * Mathf.Pow((float)U1_prime_magnitude_cached, EddyCurrentsVoltageExponent));
        statorIronLosses = Mathf.Max(0f, statorIronLosses);

        float rotorIronLosses = (IronLossesCoefficientHysteresisRotor * rotorCurrentFrequency * Mathf.Pow((float)U1_prime_magnitude_cached, HysteresisVoltageExponent) +
                                 IronLossesCoefficientEddyCurrentsRotor * rotorCurrentFrequency * rotorCurrentFrequency * Mathf.Pow((float)U1_prime_magnitude_cached, EddyCurrentsVoltageExponent));
        rotorIronLosses = Mathf.Max(0f, rotorIronLosses);

        float totalMechanicalLossesPower = totalMechanicalTorqueLosses * mechanicalAngularSpeed;
        totalMechanicalLossesPower = Mathf.Max(0f, totalMechanicalLossesPower); 

        float brushLossesPowerActual = BrushLossesPower;
        float harmonicLosses = HarmonicLossesCoefficient * currentStatorCurrentMagnitude * currentStatorCurrentMagnitude;
        if (IsInverterSupplied)
        {
            harmonicLosses += InverterHarmonicLossCoefficient * currentStatorCurrentMagnitude * currentStatorCurrentMagnitude;
        }
        if (EnableWindingHarmonicLosses)
        {
            harmonicLosses += WindingHarmonicLossesFactor * currentStatorCurrentMagnitude * currentStatorCurrentMagnitude;
        }

        harmonicLosses = Mathf.Max(0f, harmonicLosses);

        float losses_to_stator_winding = statorCopperLosses;
        float losses_to_stator_core = statorIronLosses;
        float losses_to_rotor_winding = rotorCopperLosses + brushLossesPowerActual;
        float losses_to_rotor_core = rotorIronLosses + totalMechanicalLossesPower * 0.5f; 
        float losses_to_housing = totalMechanicalLossesPower * 0.5f + harmonicLosses; 

        float G_Housing_Ambient_Convection = G_Housing_Ambient_Base + G_Housing_Ambient_SpeedFactor * Mathf.Sqrt(Mathf.Abs(mechanicalAngularSpeed));
        
        float G_Housing_Ambient_Radiation_Calc = 0f;
        if (T_housing != AmbientTemperature)
        {
             G_Housing_Ambient_Radiation_Calc = G_Housing_Ambient_RadiationFactor * ((float)System.Math.Pow(T_housing + 273.15, 4) - (float)System.Math.Pow(AmbientTemperature + 273.15, 4)) / 
                                              (T_housing - AmbientTemperature);
        }
        float G_Housing_Ambient = G_Housing_Ambient_Convection + Mathf.Max(0f, G_Housing_Ambient_Radiation_Calc); 

        float G_StatorWinding_Ambient = G_StatorWinding_Ambient_Base + G_Housing_Ambient_SpeedFactor * Mathf.Sqrt(Mathf.Abs(mechanicalAngularSpeed)) * 0.5f; 
        float G_RotorWinding_Ambient = G_RotorWinding_Ambient_Base + G_Housing_Ambient_SpeedFactor * Mathf.Sqrt(Mathf.Abs(mechanicalAngularSpeed)) * 0.5f; 
        
        float G_RotorStatorAirGap = 0.5f; 

        float dT_sw_dt = (losses_to_stator_winding - G_StatorWinding_StatorCore * (T_stator_winding - T_stator_core) - G_StatorWinding_Ambient * (T_stator_winding - AmbientTemperature)) / StatorWindingThermalCapacity;
        float dT_sc_dt = (losses_to_stator_core + G_StatorWinding_StatorCore * (T_stator_winding - T_stator_core) - G_StatorCore_Housing * (T_stator_core - T_housing)) / StatorCoreThermalCapacity;
        float dT_rw_dt = (losses_to_rotor_winding - G_RotorWinding_RotorCore * (T_rotor_winding - T_rotor_core) - G_RotorWinding_Ambient * (T_rotor_winding - AmbientTemperature) - G_RotorStatorAirGap * (T_rotor_winding - T_stator_core)) / RotorWindingThermalCapacity; 
        float dT_rc_dt = (losses_to_rotor_core + G_RotorWinding_RotorCore * (T_rotor_winding - T_rotor_core) - G_RotorCore_Housing * (T_rotor_core - T_housing) - G_RotorStatorAirGap * (T_rotor_core - T_stator_core)) / RotorCoreThermalCapacity; 
        float dT_h_dt = (losses_to_housing + G_StatorCore_Housing * (T_stator_core - T_housing) + G_RotorCore_Housing * (T_rotor_core - T_housing) - G_Housing_Ambient * (T_housing - AmbientTemperature)) / HousingThermalCapacity;

        T_stator_winding += dT_sw_dt * deltaTime;
        T_stator_core += dT_sc_dt * deltaTime;
        T_rotor_winding += dT_rw_dt * deltaTime;
        T_rotor_core += dT_rc_dt * deltaTime;
        T_housing += dT_h_dt * deltaTime;

        if (EnableThermalGradients)
        {
            T_stator_winding_max = T_stator_winding + losses_to_stator_winding * StatorWindingInternalThermalResistance;
            T_rotor_winding_max = T_rotor_winding + losses_to_rotor_winding * RotorWindingInternalThermalResistance;
        }
        else
        {
            T_stator_winding_max = T_stator_winding;
            T_rotor_winding_max = T_rotor_winding;
        }

        if (EnableInsulationAging)
        {
            if (T_stator_winding_max > InsulationAgingReferenceTemp)
            {
                float agingRate = ArrheniusConstantA * Mathf.Exp(-ArrheniusActivationEnergy_eV / (BoltzmannConstant_eV_K * (T_stator_winding_max + 273.15f)));
                _insulationLifeConsumed += agingRate * deltaTime * 100f;
                _insulationLifeConsumed = Mathf.Min(_insulationLifeConsumed, 100f);
            }
        }

        if (EnableVibrationNoiseModel)
        {
            _currentVibrationLevel = VibrationAccelerationFactor * Mathf.Abs(_lastAngularAcceleration) + 
                                     InverterTorqueRippleFactor * Mathf.Abs(_inverterTorqueRipple);
            _currentNoiseLevel = NoiseSpeedFactor * Mathf.Abs(mechanicalAngularSpeed) + 
                                 NoiseLoadFactor * Mathf.Abs(_currentLoadTorque) + 
                                 InverterTorqueRippleFactor * Mathf.Abs(_inverterTorqueRipple);
        }

        if (EnableBearingWear)
        {
            _bearingWearLevel += BearingWearRateFactor * _currentVibrationLevel * deltaTime;
            _bearingWearLevel = Mathf.Min(_bearingWearLevel, 100f);
        }

        float totalLosses = statorCopperLosses + rotorCopperLosses + statorIronLosses + rotorIronLosses + totalMechanicalLossesPower + brushLossesPowerActual + harmonicLosses;
        
        float electromagneticPower = currentElectromagneticTorque * mechanicalAngularSpeed;

        float usefulMechanicalPower = 0f;
        float electricalInputPower = 0f;
        float mechanicalInputPower = 0f;
        float electricalOutputPower = 0f;

        if (electromagneticPower >= 0)
        {
            usefulMechanicalPower = currentElectromagneticTorque * mechanicalAngularSpeed - totalMechanicalLossesPower - (_currentLoadTorque * mechanicalAngularSpeed);
            usefulMechanicalPower = Mathf.Max(0f, usefulMechanicalPower);
            electricalInputPower = usefulMechanicalPower + totalLosses;
            if (electricalInputPower < 1e-6f) electricalInputPower = 1e-6f;
        }
        else
        {
            mechanicalInputPower = Mathf.Abs(_currentLoadTorque * mechanicalAngularSpeed); 
            if (mechanicalAngularSpeed != 0f)
            {
                mechanicalInputPower = Mathf.Abs(electromagneticPower) + totalMechanicalLossesPower; 
            } else {
                mechanicalInputPower = totalMechanicalLossesPower;
            }
            mechanicalInputPower = Mathf.Max(0f, mechanicalInputPower);

            electricalOutputPower = Mathf.Abs(electromagneticPower) - (statorCopperLosses + statorIronLosses + harmonicLosses);
            electricalOutputPower = Mathf.Max(0f, electricalOutputPower);

            if (mechanicalInputPower < 1e-6f) mechanicalInputPower = 1e-6f;
        }

        float efficiency = 0f;
        if (electromagneticPower >= 0)
        {
            if (electricalInputPower > 1e-6f)
            {
                efficiency = usefulMechanicalPower / electricalInputPower;
            }
        }
        else
        {
            if (mechanicalInputPower > 1e-6f)
            {
                 efficiency = electricalOutputPower / mechanicalInputPower;
            }
        }
        efficiency = Mathf.Clamp01(efficiency);

        if (LogStartUpData)
        {
            logTimer += deltaTime;
            if (logTimer >= LogInterval)
            {
                startupTime.Add(currentTime);
                startupSpeedRPM.Add(GetMechanicalSpeedRPM());
                startupTorque.Add(currentElectromagneticTorque);
                startupStatorCurrent.Add(currentStatorCurrentMagnitude);
                startupStatorTempWinding.Add(T_stator_winding_max);
                logTimer = 0f;
            }
            if (currentTime > 15f && Mathf.Abs(GetMechanicalSpeedRPM() - GetSynchronousSpeedRPM()) < 10 && startupSpeedRPM.Count > 100)
            {
                LogStartUpData = false;
                Debug.Log("Startup logging finished. Data collected.");
                PrintStartupDataSummary();
            }
        }
        
        if (EnableRealtimeConsoleLogging)
        {
            _realtimeLogTimer += deltaTime;
            if (_realtimeLogTimer >= RealtimeLogInterval)
            {
                string logMessage = $"Time: {currentTime:F2}s | " +
                                    $"Line V: {InputLineVoltage:F2}V | " + // Используем InputLineVoltage
                                    $"Freq: {InputFrequency:F2}Hz | " +    // Используем InputFrequency
                                    $"V_A: {voltagePhaseAMagnitude:F2}V ({voltagePhaseAAngleDeg:F1}°) | " +
                                    $"I_A: {currentPhaseAMagnitude:F2}A ({currentPhaseAAngleDeg:F1}°) | " +
                                    $"V_AB: {airGapVoltagePhaseAMagnitude:F2}V ({airGapVoltagePhaseAAngleDeg:F1}°) | " +
                                    $"I_rot_AB: {rotorCurrentPhaseAMagnitudeReferred:F2}A ({rotorCurrentPhaseAAngleDegReferred:F1}°) | " +
                                    $"Speed: {GetMechanicalSpeedRPM():F2} RPM | " +
                                    $"Torque: {currentElectromagneticTorque:F2} Nm | " +
                                    $"Slip: {GetCurrentSlip():F3} | " +
                                    $"Eff: {efficiency * 100:F2}% | " +
                                    $"Temp(SW_Surf): {T_stator_winding:F1}°C | " +
                                    $"Temp(SW_Max): {T_stator_winding_max:F1}°C | " +
                                    $"Temp(RW_Surf): {T_rotor_winding:F1}°C | " +
                                    $"Temp(RW_Max): {T_rotor_winding_max:F1}°C | " +
                                    $"Temp(H): {T_housing:F1}°C | " +
                                    $"Insulation Life Consumed: {_insulationLifeConsumed:F2}% | " +
                                    $"Bearing Wear: {_bearingWearLevel:F2}% | " +
                                    $"Vibration: {_currentVibrationLevel:F4} | " +
                                    $"Noise: {_currentNoiseLevel:F4} | " +
                                    $"Xm: {current_Xm:F2}";
                
                Debug.Log(logMessage);
                _realtimeLogTimer = 0f;
            }
        }
    }

    // Метод для нормализации угла в диапазоне (-180, 180]
    private void NormalizeAngle(ref float angle)
    {
        while (angle <= -180f) angle += 360f;
        while (angle > 180f) angle -= 360f;
    }

    // Метод для сброса состояния симуляции при обнаружении NaN/Infinity
    private void ResetSimulationState()
    {
        mechanicalAngularSpeed = 0f;
        rotorAngle = 0f;
        currentElectromagneticTorque = 0f;
        currentStatorCurrentMagnitude = 0f;
        _lastAngularAcceleration = 0f;

        // Сброс температур до начальных значений
        T_stator_winding = AmbientTemperature;
        T_stator_core = AmbientTemperature;
        T_rotor_winding = AmbientTemperature;
        T_rotor_core = AmbientTemperature;
        T_housing = AmbientTemperature;
        T_stator_winding_max = AmbientTemperature;
        T_rotor_winding_max = AmbientTemperature;

        _insulationLifeConsumed = 0f;
        _bearingWearLevel = 0f;

        // Можно также сбросить график нагрузки
        _currentLoadTorque = LoadTorque;
        // _currentLineVoltage = InputLineVoltage; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО

        // Переинициализация электрических параметров
        current_R1 = GetTemperatureCompensatedResistance(R1_Stator_Ref, T_stator_winding);
        current_X1 = GetTemperatureCompensatedReactance(X1_Stator, T_stator_winding); 
        current_R2_Effective = GetTemperatureCompensatedResistance(R2_Rotor_Ref, T_rotor_winding) + ExternalRotorResistance;
        current_X2_Effective = GetTemperatureCompensatedReactance(X2_Rotor_Ref, T_rotor_winding); 
        current_Xm = Xm_Magnetizing_Unsaturated; // Инициализация Xm

        // Сброс всех фазных параметров
        voltagePhaseAMagnitude = 0f;
        voltagePhaseAAngleDeg = 0f;
        voltagePhaseBMagnitude = 0f;
        voltagePhaseBAngleDeg = 0f;
        voltagePhaseCMagnitude = 0f;
        voltageCAngleDeg = 0f; // Corrected initialization

        currentPhaseAMagnitude = 0f;
        currentPhaseAAngleDeg = 0f;
        currentPhaseBMagnitude = 0f;
        currentPhaseBAngleDeg = 0f;
        currentPhaseCMagnitude = 0f;
        currentPhaseCAngleDeg = 0f;

        airGapVoltagePhaseAMagnitude = 0f;
        airGapVoltagePhaseAAngleDeg = 0f;
        airGapVoltagePhaseBMagnitude = 0f;
        airGapVoltagePhaseBAngleDeg = 0f;
        airGapVoltagePhaseCMagnitude = 0f;
        airGapVoltageCAngleDeg = 0f; // Corrected initialization

        rotorCurrentPhaseAMagnitudeReferred = 0f;
        rotorCurrentPhaseAAngleDegReferred = 0f;
        rotorCurrentPhaseBMagnitudeReferred = 0f;
        rotorCurrentPhaseBAngleDegReferred = 0f;
        rotorCurrentPhaseCMagnitudeReferred = 0f;
        rotorCurrentPhaseCAngleDegReferred = 0f;
    }


    private float GetTemperatureCompensatedResistance(float R_ref, float temperature)
    {
        return R_ref * (1f + AlphaCopper * (temperature - ReferenceTemperature));
    }

    private float GetTemperatureCompensatedReactance(float X_ref, float temperature)
    {
        return X_ref * (1f + AlphaReactance * (temperature - ReferenceTemperature));
    }

    private float ApplySkinEffectToResistance(float R_base, float rotorFreq)
    {
        float normalizedFreq = rotorFreq / InputFrequency;
        return R_base * (1f + 0.5f * Mathf.Sqrt(normalizedFreq)); 
    }

    private float ApplySkinEffectToReactance(float X_base, float rotorFreq)
    {
        float normalizedFreq = rotorFreq / InputFrequency;
        return X_base * (1f - 0.1f * Mathf.Sqrt(normalizedFreq));
    }

    private float CalculateSaturatedXm(float U_phase, float R1, float X1, float R2_eff, float X2_eff, float slip)
    {
        float Xm_calculated = Xm_Magnetizing_Unsaturated;

        float R2_prime_slip_temp = R2_eff / slip;
        Complex Z_rotor_branch_temp = new Complex((double)R2_prime_slip_temp, (double)X2_eff);
        Complex Z_magnetizing_temp = new Complex(0.0, (double)Xm_Magnetizing_Unsaturated); 
        
        Complex sum_Z_branches_temp = Z_rotor_branch_temp + Z_magnetizing_temp;
        Complex Z_parallel_temp;
        if (sum_Z_branches_temp.Magnitude < 1e-12)
        {
            Z_parallel_temp = _zeroComplex; 
        }
        else
        {
            Z_parallel_temp = (Z_rotor_branch_temp * Z_magnetizing_temp) / sum_Z_branches_temp;
        }

        Complex Z_total_temp = new Complex((double)R1, (double)X1) + Z_parallel_temp;
        
        if (Z_total_temp.Magnitude < 1e-12) return Xm_Magnetizing_Unsaturated; // Ультра-малый эпсилон

        Complex I1_temp = new Complex((double)U_phase, 0.0) / Z_total_temp;
        Complex U1_prime_temp = I1_temp * Z_parallel_temp;
        
        float U1_prime_magnitude = (float)U1_prime_temp.Magnitude;

        if (UsePiecewiseSaturation && SaturationCurveXm != null && SaturationCurveXm.length > 0)
        {
            Xm_calculated = SaturationCurveXm.Evaluate(U1_prime_magnitude);
        }
        else
        {
            if (U1_prime_magnitude > Xm_SaturationVoltageThreshold)
            {
                float saturationRatio = (U1_prime_magnitude - Xm_SaturationVoltageThreshold) / Xm_SaturationVoltageThreshold;
                float reductionFactor = 1f - Mathf.Min(0.9f, saturationRatio * Xm_SaturationFactor);
                Xm_calculated = Xm_Magnetizing_Unsaturated * reductionFactor;
            }
        }
        
        return Mathf.Max(0.1f, Xm_calculated); // Убедимся, что Xm всегда положительный и не слишком маленький
    }

    private float ApplyThermalExpansionToXm(float Xm_base, float coreTemperature)
    {
        if (coreTemperature > ThermalExpansionReferenceTemp)
        {
            return Xm_base * (1f - ThermalExpansionCoefficientXm * (coreTemperature - ThermalExpansionReferenceTemp));
        }
        else if (coreTemperature < ThermalExpansionReferenceTemp)
        {
            return Xm_base * (1f + ThermalExpansionCoefficientXm * 0.5f * (ThermalExpansionReferenceTemp - coreTemperature)); 
        }
        return Xm_base;
    }

    private void HandleStartingScheme(float currentTime)
    {
        float appliedPhaseVoltageFactor = 1.0f;
        // ExternalRotorResistance теперь будет управляться только FrequencyController или вручную.
        // Здесь мы больше не изменяем его на основе StarDelta/RotorResistance схем
        // ExternalRotorResistance = 0.0f; // Убрано, чтобы не конфликтовать с FrequencyController

        switch (SelectedStartingScheme)
        {
            case StartingScheme.DirectOnLine:
                appliedPhaseVoltageFactor = 1.0f;
                break;
            case StartingScheme.StarDelta:
                if (currentTime < StarDeltaSwitchTime)
                {
                    appliedPhaseVoltageFactor = _invSqrt3; 
                }
                else
                {
                    appliedPhaseVoltageFactor = 1.0f;
                }
                break;
            case StartingScheme.RotorResistance:
                // В этом случае ExternalRotorResistance управляется FrequencyController
                // или остается тем, что установлено в инспекторе.
                // Не меняем ExternalRotorResistance здесь.
                appliedPhaseVoltageFactor = 1.0f;
                break;
            case StartingScheme.AutoTransformer:
                if (currentTime < AutoTransformerSwitchTime)
                {
                    appliedPhaseVoltageFactor = AutoTransformerRatio;
                }
                else
                {
                    appliedPhaseVoltageFactor = 1.0f;
                }
                break;
        }
        phaseVoltageActual = phaseVoltageSource * appliedPhaseVoltageFactor;
        if (float.IsNaN(phaseVoltageActual) || float.IsInfinity(phaseVoltageActual))
        {
            Debug.LogError($"[NaN Check] phaseVoltageActual became NaN/Infinity. Value: {phaseVoltageActual}. Resetting to initial. Time: {currentTime:F2}");
            phaseVoltageActual = InputLineVoltage * _invSqrt3; // Используем InputLineVoltage
        }
    }

    private void HandleDynamicLoadChanges(float currentTime)
    {
        if (!EnableLoadChangeSchedule || LoadChangeTimes == null || TargetLoadTorques == null || _currentLoadScheduleIndex >= LoadChangeTimes.Length)
        {
            _currentLoadTorque = _initialLoadTorque;
            return;
        }
        while (_currentLoadScheduleIndex < LoadChangeTimes.Length && currentTime >= LoadChangeTimes[_currentLoadScheduleIndex])
        {
            _currentLoadTorque = TargetLoadTorques[_currentLoadScheduleIndex];
            Debug.Log($"Load changed to {_currentLoadTorque:F2} Nm at {currentTime:F2}s (scheduled time: {LoadChangeTimes[_currentLoadScheduleIndex]:F2}s)");
            _currentLoadScheduleIndex++;
        }
    }

    // private void HandleDynamicVoltageChanges(float currentTime) // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // {                                                           // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     if (!EnableVoltageChangeSchedule || VoltageChangeTimes == null || TargetLineVoltages == null || _currentVoltageScheduleIndex >= VoltageChangeTimes.Length) // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     {                                                           // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //         _currentLineVoltage = _initialLineVoltage;          // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //         return;                                             // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     }                                                           // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     while (_currentVoltageScheduleIndex < VoltageChangeTimes.Length && currentTime >= VoltageChangeTimes[_currentVoltageScheduleIndex]) // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     {                                                           // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //         _currentLineVoltage = TargetLineVoltages[_currentVoltageScheduleIndex]; // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //         Debug.Log($"Line Voltage changed to {_currentLineVoltage:F2} V at {currentTime:F2}s (scheduled time: {VoltageChangeTimes[_currentVoltageScheduleIndex]:F2}s)"); // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //         _currentVoltageScheduleIndex++;                         // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    //     }                                                           // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО
    // }                                                               // <-- ЗАКОММЕНТИРОВАНО/УДАЛЕНО

    public float GetMechanicalSpeedRPM()
    {
        return mechanicalAngularSpeed * _pi30Div; 
    }

    public float GetElectromagneticTorque()
    {
        return currentElectromagneticTorque;
    }

    public float GetStatorCurrent()
    {
        return currentStatorCurrentMagnitude; 
    }

    public float GetStatorWindingTemperature()
    {
        return T_stator_winding;
    }
    public float GetStatorCoreTemperature() { return T_stator_core; }
    public float GetRotorWindingTemperature() { return T_rotor_winding; }
    public float GetRotorCoreTemperature() { return T_rotor_core; }
    public float GetHousingTemperature() { return T_housing; }
    public float GetStatorWindingMaxTemperature() { return T_stator_winding_max; }
    public float GetRotorWindingMaxTemperature() { return T_rotor_winding_max; }
    public float GetInsulationLifeConsumed() { return _insulationLifeConsumed; }
    public float GetBearingWearLevel() { return _bearingWearLevel; }
    public float GetCurrentVibrationLevel() { return _currentVibrationLevel; }
    public float GetCurrentNoiseLevel() { return _currentNoiseLevel; }
    public float GetCurrentLineVoltage() { return InputLineVoltage; } // Теперь берем из InputLineVoltage

    public float GetCurrentSlip()
    {
        if (Mathf.Abs(synchronousMechanicalSpeed) < 1e-6f) return 1.0f;
        float slip = (synchronousMechanicalSpeed - mechanicalAngularSpeed) / synchronousMechanicalSpeed;
        if (float.IsNaN(slip) || float.IsInfinity(slip)) return 1.0f; 
        if (Mathf.Abs(slip) < MIN_SLIP_EPSILON)
        {
            slip = Mathf.Sign(slip) * MIN_SLIP_EPSILON;
            if (slip == 0) slip = MIN_SLIP_EPSILON; 
        }
        return slip;
    }

    public float GetSynchronousSpeedRPM(float frequencyHz) 
    {
        if (PolePairs == 0) 
        {
            Debug.LogWarning("PolePairs is zero, synchronous speed cannot be calculated.");
            return 0f;
        }
        // Ns (об/мин) = (120 * f) / p, где p - количество полюсов (НЕ пар полюсов)
        // Так как PolePairs - это количество пар полюсов, то общее количество полюсов = PolePairs * 2
        return (120f * frequencyHz) / (PolePairs * 2f); 
    }

     public float GetSynchronousSpeedRPM() 
    {
        return GetSynchronousSpeedRPM(InputFrequency);
    }

    public float GetSynchronousSpeedRadPerSec()
    {
        return GetSynchronousSpeedRadPerSec(InputFrequency);
    }

    public float GetSynchronousSpeedRadPerSec(float frequencyHz)
    {
        if (PolePairs == 0) return 0f;
        return (2f * Mathf.PI * frequencyHz) / PolePairs;
    }

    // ----- Public Setter Methods (для FrequencyController) -----
    public float InputFrequency
    {
        get { return InputFrequency; }
        set { InputFrequency = value; }
    }

    public float InputLineVoltage
    {
        get { return InputLineVoltage; }
        set { InputLineVoltage = value; }
    }

    public float GetEfficiency()
    {
        float current_speed_eff = mechanicalAngularSpeed;
        float current_slip_eff = GetCurrentSlip();
        float rotorCurrentFrequency_eff = Mathf.Abs(current_slip_eff * InputFrequency);

        float R1_calc_eff = GetTemperatureCompensatedResistance(R1_Stator_Ref, T_stator_winding);
        float X1_calc_eff = GetTemperatureCompensatedReactance(X1_Stator, T_stator_winding);
        // Используем GetTemperatureCompensatedResistance и GetTemperatureCompensatedReactance напрямую здесь
        // А также ApplySkinEffectToResistance и ApplySkinEffectToReactance, чтобы не зависеть от current_R2_Effective и current_X2_Effective
        // Это предотвратит ошибки при вызове GetEfficiency до того, как FixedUpdate рассчитает current_X2_Effective
        float R2_Effective_calc_eff = ApplySkinEffectToResistance(GetTemperatureCompensatedResistance(R2_Rotor_Ref, T_rotor_winding), rotorCurrentFrequency_eff) + ExternalRotorResistance;
        float X2_Effective_calc_eff = ApplySkinEffectToReactance(GetTemperatureCompensatedReactance(X2_Rotor_Ref, T_rotor_winding), rotorCurrentFrequency_eff);
        
        float Xm_calc_eff = CalculateSaturatedXm(voltagePhaseAMagnitude, R1_calc_eff, X1_calc_eff, R2_Effective_calc_eff, X2_Effective_calc_eff, current_slip_eff); 
        if (EnableThermalExpansionEffect)
        {
            Xm_calc_eff = ApplyThermalExpansionToXm(Xm_calc_eff, T_stator_core);
        }

        float R2_prime_slip_eff = R2_Effective_calc_eff / current_slip_eff;
        Complex Z_rotor_branch_eff = new Complex((double)R2_prime_slip_eff, (double)X2_Effective_calc_eff);
        Complex Z_magnetizing_eff = new Complex(0.0, (double)Xm_calc_eff);
        
        Complex sum_Z_branches_eff = Z_rotor_branch_eff + Z_magnetizing_eff;
        Complex Z_parallel_eff;
        if (sum_Z_branches_eff.Magnitude < 1e-12)
        {
            Z_parallel_eff = _zeroComplex;
        }
        else
        {
            Z_parallel_eff = (Z_rotor_branch_eff * Z_magnetizing_eff) / sum_Z_branches_eff;
        }

        Complex Z_total_motor_eff = new Complex((double)R1_calc_eff, (double)X1_calc_eff) + Z_parallel_eff;

        // Используем параметры фазы А, так как они представляют общие электрические характеристики
        Complex actualPhaseVoltageAtTerminals_eff_calc = new Complex((double)voltagePhaseAMagnitude, (double)voltagePhaseAAngleDeg * Mathf.Deg2Rad); 
        
        if (Z_total_motor_eff.Magnitude < 1e-12) return 0f;

        Complex I1_eff = actualPhaseVoltageAtTerminals_eff_calc / Z_total_motor_eff;
        Complex U1_prime_eff = I1_eff * Z_parallel_eff;
        
        Complex I2_prime_eff;
        if (Z_rotor_branch_eff.Magnitude < 1e-12)
        {
            I2_prime_eff = _zeroComplex;
        } else { I2_prime_eff = U1_prime_eff / Z_rotor_branch_eff; }


        float statorCopperLosses_eff = 3f * (float)I1_eff.Magnitude * (float)I1_eff.Magnitude * R1_calc_eff;
        float rotorCopperLosses_eff = (float)(3f * System.Math.Pow(I2_prime_eff.Magnitude, 2) * R2_Effective_calc_eff);

        float U1_prime_magnitude_eff_cached = (float)U1_prime_eff.Magnitude;
        float statorIronLosses_eff = (IronLossesCoefficientHysteresisStator * InputFrequency * Mathf.Pow((float)U1_prime_magnitude_eff_cached, HysteresisVoltageExponent) + 
                                  IronLossesCoefficientEddyCurrentsStator * InputFrequency * InputFrequency * Mathf.Pow((float)U1_prime_magnitude_eff_cached, EddyCurrentsVoltageExponent));
        statorIronLosses_eff = Mathf.Max(0f, statorIronLosses_eff);

        float rotorIronLosses_eff = (IronLossesCoefficientHysteresisRotor * rotorCurrentFrequency_eff * Mathf.Pow((float)U1_prime_magnitude_eff_cached, HysteresisVoltageExponent) +
                                 IronLossesCoefficientEddyCurrentsRotor * rotorCurrentFrequency_eff * rotorCurrentFrequency_eff * Mathf.Pow((float)U1_prime_magnitude_eff_cached, EddyCurrentsVoltageExponent));
        rotorIronLosses_eff = Mathf.Max(0f, rotorIronLosses_eff);

        float frictionTorque_eff = FrictionCoefficientLinear * current_speed_eff + FrictionCoefficientQuadratic * current_speed_eff * current_speed_eff;
        float bearingLossesTorque_eff = BearingLossesConstant + BearingLossesCoefficient * Mathf.Abs(current_speed_eff);
        if (EnableBearingWear)
        {
            bearingLossesTorque_eff *= (1f + _bearingWearLevel / 100f * BearingWearLossIncreaseFactor);
        }
        float ventilationLossesTorque_eff = VentilationLossesCubicCoefficient * Mathf.Pow(Mathf.Abs(current_speed_eff), 3);
        float loadDependentFrictionTorque_eff = LoadDependentFrictionCoefficient * Mathf.Abs(_currentLoadTorque);
        float totalMechanicalTorqueLosses_calc_eff = frictionTorque_eff + bearingLossesTorque_eff + ventilationLossesTorque_eff + loadDependentFrictionTorque_eff;

        float totalMechanicalLossesPower_eff = totalMechanicalTorqueLosses_calc_eff * current_speed_eff; 
        totalMechanicalLossesPower_eff = Mathf.Max(0f, totalMechanicalLossesPower_eff);

        float brushLossesPowerActual_eff = BrushLossesPower;
        float harmonicLosses_eff = HarmonicLossesCoefficient * (float)I1_eff.Magnitude * (float)I1_eff.Magnitude;
        if (IsInverterSupplied)
        {
            harmonicLosses_eff += InverterHarmonicLossCoefficient * (float)I1_eff.Magnitude * (float)I1_eff.Magnitude;
        }
        if (EnableWindingHarmonicLosses)
        {
            harmonicLosses_eff += WindingHarmonicLossesFactor * (float)I1_eff.Magnitude * (float)I1_eff.Magnitude;
        }
        harmonicLosses_eff = Mathf.Max(0f, harmonicLosses_eff);

        float totalLosses_eff = statorCopperLosses_eff + rotorCopperLosses_eff + statorIronLosses_eff + rotorIronLosses_eff + totalMechanicalLossesPower_eff + brushLossesPowerActual_eff + harmonicLosses_eff;
        
        float electromagneticPower_eff = GetElectromagneticTorque() * current_speed_eff; 

        float usefulMechanicalPower_eff = 0f;
        float electricalInputPower_eff = 0f;
        float mechanicalInputPower_eff = 0f;
        float electricalOutputPower_eff = 0f;

        if (electromagneticPower_eff >= 0)
        {
            usefulMechanicalPower_eff = electromagneticPower_eff - totalMechanicalLossesPower_eff - (_currentLoadTorque * current_speed_eff);
            usefulMechanicalPower_eff = Mathf.Max(0f, usefulMechanicalPower_eff);
            electricalInputPower_eff = usefulMechanicalPower_eff + totalLosses_eff;
            
            if (electricalInputPower_eff < 1e-6f) electricalInputPower_eff = 1e-6f;
            return Mathf.Clamp01(usefulMechanicalPower_eff / electricalInputPower_eff);
        }
        else
        {
            mechanicalInputPower_eff = Mathf.Abs(_currentLoadTorque * current_speed_eff); 
            if (current_speed_eff != 0f)
            {
                mechanicalInputPower_eff = Mathf.Abs(electromagneticPower_eff) + totalMechanicalLossesPower_eff;
            } else {
                mechanicalInputPower_eff = totalMechanicalLossesPower_eff;
            }
            mechanicalInputPower_eff = Mathf.Max(0f, mechanicalInputPower_eff);

            electricalOutputPower_eff = Mathf.Abs(electromagneticPower_eff) - (statorCopperLosses_eff + statorIronLosses_eff + harmonicLosses_eff);
            electricalOutputPower_eff = Mathf.Max(0f, electricalOutputPower_eff);
            
            if (mechanicalInputPower_eff < 1e-6f) mechanicalInputPower_eff = 1e-6f;
            return Mathf.Clamp01(electricalOutputPower_eff / mechanicalInputPower_eff);
        }
    }


    private void PrintStartupDataSummary()
    {
        if (startupTime.Count == 0)
        {
            Debug.Log("No startup data collected.");
            return;
        }

        Debug.Log("--- Startup Data Summary ---");
        Debug.Log($"Total data points: {startupTime.Count}");
        Debug.Log($"Duration: {startupTime.Last():F2} s");
        Debug.Log($"Peak Speed: {startupSpeedRPM.Max():F2} RPM at {startupTime[startupSpeedRPM.IndexOf(startupSpeedRPM.Max())]:F2} s");
        Debug.Log($"Peak Torque: {startupTorque.Max():F2} Nm at {startupTime[startupTorque.IndexOf(startupTorque.Max())]:F2} s");
        Debug.Log($"Peak Stator Current: {startupStatorCurrent.Max():F2} A at {startupTime[startupStatorCurrent.IndexOf(startupStatorCurrent.Max())]:F2} s");
        Debug.Log($"Final Stator Winding Temperature (Max): {startupStatorTempWinding.Last():F2}°C");
        Debug.Log($"Final Insulation Life Consumed: {_insulationLifeConsumed:F2}%");
        Debug.Log($"Final Bearing Wear Level: {_bearingWearLevel:F2}%");
        Debug.Log("----------------------------");
    }
}