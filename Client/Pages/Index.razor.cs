using Microsoft.AspNetCore.Components;
using Microsoft.JSInterop;
using System.Numerics;
using System.Text.Json;
using Animations.Client.Enums;
using Animations.Client.Extensions;
using Animations.Client.Models.Parameters;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{
    private bool _isVisualizationInitialized;
    private DateTime _lastFrameTime = DateTime.UtcNow;
    private CancellationTokenSource _simLoopCancel = new();
    private SimManager? _simulationManager;

    #region View-bound Properties

    private bool DebugMode { get; set; }
    private string DebugInformation { get; set; } = string.Empty;
    private int Divisions { get; set; } = 10;
    private int VoxelsPerDivision { get; set; } = 10;
    private float TimeStep { get; set; } = 0.05f;
    private float Gravity { get; set; } = -9.81f;
    private float SingleSimulationExtent { get; set; } = 10f;
    private bool ShowCoils { get; set; }
    private bool ShowGravityField { get; set; }
    private bool ShowMagneticField { get; set; }
    private int MaxFramesPerSecond { get; set; } = 30;
    private int FrameCount { get; set; }
    private float FrameRate { get; set; }
    private bool Paused { get; set; } = true;
    private bool AreControlsVisible { get; set; } = true;
    private bool ShowManualInputs { get; set; }
    private string FieldDrawingType { get; set; } = "colorMapping";
    private float AmbientLightLevel { get; set; } = 1;
    private float CeilingLightLevel { get; set; } = 1;
    private string SimulationModeString { get; set; } = "DipoleApproximation";

    #endregion

    #region View-bound Actions

    private void ToggleDebugMode() => DebugMode = !DebugMode;

    private async Task ToggleMagnetDrawingStyle()
    {
        ShowCoils = !ShowCoils;
        await UpdateClientVisualization();
    }

    private async Task ToggleGravityFieldVisibility()
    {
        ShowGravityField = !ShowGravityField;
        if (Paused)
        {
            _simulationManager?.RecalculateFields(GetSimulationParametersFromView());
            await UpdateClientVisualization();
        }
    }

    private async Task ToggleMagneticFieldVisibility()
    {
        ShowMagneticField = !ShowMagneticField;
        if (Paused)
        {
            _simulationManager?.RecalculateFields(GetSimulationParametersFromView());
            await UpdateClientVisualization();
        }
    }

    private void TogglePaused() => Paused = !Paused;

    private void ToggleManualInputs() => ShowManualInputs = !ShowManualInputs;

    private void ToggleControlsVisibility() => AreControlsVisible = !AreControlsVisible;

    private string GetControlPanelClass() => AreControlsVisible ? "expanded" : "collapsed";

    #endregion

    protected override async Task OnAfterRenderAsync(bool firstRender)
    {
        if (firstRender)
        {
            await InitializeClientVisualization(SingleSimulationExtent);
            await RestartSimulation();
        }
    }

    #region Simulation Management

    private async Task RestartSimulation()
    {
        CancelSimulation();
        ResetSimulator();
        await UpdateClientVisualization();
        await InvokeAsync(StateHasChanged);
        await StartDrawSimulationLoop(_simLoopCancel.Token);
    }

    private void ResetSimulator()
    {
        FrameCount = 0;
        FrameRate = 0;

        _simulationManager = new SimManager(GetSimulationParametersFromView());
    }

    private void CancelSimulation()
    {
        _simLoopCancel.Cancel();
        _simLoopCancel.Dispose();
        _simLoopCancel = new CancellationTokenSource();
    }

    private SimulationParameters GetSimulationParametersFromView()
    {
        return new SimulationParameters
        {
            TimeStep = TimeStep,
            Gravity = new Vector3(0, Gravity, 0),
            SimulationExtents = new Vector3(SingleSimulationExtent, SingleSimulationExtent, SingleSimulationExtent),
            SimulationMode = Enum.Parse<SimulationMode>(SimulationModeString),
            Divisions = Divisions,
            VoxelsPerDivision = VoxelsPerDivision,
            ShowGravityField = ShowGravityField,
            ShowMagneticField = ShowMagneticField,
        };
    }

    private async Task StartDrawSimulationLoop(CancellationToken simLoopCancelToken)
    {
        while (!simLoopCancelToken.IsCancellationRequested)
        {
            if (Paused)
                await Task.Delay(100, simLoopCancelToken); // Small delay to reduce CPU usage
            else
                await Task.Run(async () =>
                {
                    _simulationManager?.UpdateSimulation(GetSimulationParametersFromView());
                    await UpdateClientVisualization();
                    await DelayUntilNextRequestedFrame(simLoopCancelToken);
                }, simLoopCancelToken);
        }
    }

    private async Task DelayUntilNextRequestedFrame(CancellationToken simLoopCancelToken)
    {
        var currentFrameTime = DateTime.UtcNow;
        var elapsed = currentFrameTime - _lastFrameTime;

        if (elapsed.TotalSeconds > 0)
            FrameRate = 1.0f / (float)elapsed.TotalSeconds;
        _lastFrameTime = currentFrameTime;
        FrameCount++;

        var difference = (int)(Math.Floor(1000.0f / MaxFramesPerSecond) - (int)elapsed.TotalMilliseconds);
        if (difference > 0)
            await Task.Delay(difference, simLoopCancelToken);
    }

    #endregion

    #region Interop with Client

    private DrawingParameters GetDrawingParametersFromView()
    {
        return new DrawingParameters
        {
            ShowCoils = ShowCoils,
            FieldDrawingStyle = FieldDrawingType,
            ShowGravityField = ShowGravityField,
            ShowMagneticField = ShowMagneticField,
            AmbientLightLevel = AmbientLightLevel,
            CeilingLightLevel = CeilingLightLevel
        };
    }

    private async Task InitializeClientVisualization(float extent)
    {
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs",
            new { width = extent, height = extent, depth = extent },
            GetDrawingParametersFromView());

        _isVisualizationInitialized = true;
    }

    private async Task UpdateClientVisualization()
    {
        if (_simulationManager == null) return;

        var state = _simulationManager.GetSimulationState(ShowGravityField, ShowMagneticField);
        state.TimeSinceStart = FrameCount * TimeStep;

        if (DebugMode)
            DebugInformation = JsonSerializer.Serialize(state, new JsonSerializerOptions { WriteIndented = true });
        else if (_isVisualizationInitialized)
            await Profiling.RunWithClockingLogAsync(
                () => JsRuntime.InvokeVoidAsync(
                    "Animations.updateThreeJsScene",
                    state,
                    GetDrawingParametersFromView()),
                "JsRuntime.InvokeVoidAsync(Animations.updateThreeJsScene");

        await InvokeAsync(StateHasChanged);
    }

    private async Task PromptToInstallApp()
    {
        var success = await JsRuntime.InvokeAsync<bool>("showPWAInstallPrompt");

        Console.WriteLine(
            $"Install prompt was {(success ? "" : "not")} accepted." +
            $"{(success ? "" : " Event to trigger it might not have been caught.")}");
    }

    #endregion
}