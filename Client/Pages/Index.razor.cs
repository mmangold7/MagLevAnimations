using Animations.Shared;
using Microsoft.AspNetCore.Components;
using Microsoft.JSInterop;
using System.Diagnostics;
using System.Numerics;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{
    private DateTime _lastFrameTime = DateTime.UtcNow;
    private CancellationTokenSource _simLoopCancel = new();
    private SimulationManager? _simulationManager;
    private bool _isVisualizationInitialized;
    private CollisionMagnetSimulation? _collisionMagnetSimulator;

    private int Divisions { get; set; } = 10;
    private int VoxelsPerDivision { get; set; } = 10;
    private float TimeStep { get; set; } = 0.05f;
    private float Gravity { get; set; } = 0f;
    private float SingleSimulationExtent { get; set; } = 10f;
    private bool ShowCoils { get; set; }
    private bool ShowGravityField { get; set; }
    private bool ShowMagneticField { get; set; }
    private bool UseDipoleApproximation { get; set; } = true;
    private int MaxFramesPerSecond { get; set; } = 30;
    private int FrameCount { get; set; }
    private float FrameRate { get; set; }
    private bool Paused { get; set; } = true;
    private bool AreControlsVisible { get; set; } = true;
    private bool ShowManualInputs { get; set; }
    private string FieldDrawingType { get; set; } = "colorMapping";
    private float AmbientLightLevel { get; set; } = 1;
    private float CeilingLightLevel { get; set; } = 1;

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
            _simulationManager?.RecalculateFields();
            await UpdateClientVisualization();
        }
    }
    private async Task ToggleMagneticFieldVisibility()
    {
        ShowMagneticField = !ShowMagneticField;
        if (Paused)
        {
            _simulationManager?.RecalculateFields();
            await UpdateClientVisualization();
        }
    }
    private void ToggleApproximationMode() => UseDipoleApproximation = !UseDipoleApproximation;
    private void TogglePaused() => Paused = !Paused;
    private void ToggleManualInputs() => ShowManualInputs = !ShowManualInputs;
    private void ToggleControlsVisibility() => AreControlsVisible = !AreControlsVisible;
    private string GetControlPanelClass() => AreControlsVisible ? "expanded" : "collapsed";
    private SimulationParameters GetSimulationParametersFromView()
    {
        return new SimulationParameters
        {
            TimeStep = TimeStep,
            Gravity = new Vector3(0, Gravity, 0),
            SimulationExtents = new Vector3(SingleSimulationExtent, SingleSimulationExtent, SingleSimulationExtent),
            Mode = UseDipoleApproximation ? SimulationMode.DipoleApproximation : SimulationMode.VoxelBased,
            Divisions = Divisions,
            VoxelsPerDivision = VoxelsPerDivision,
            ShowGravityField = ShowGravityField,
            ShowMagneticField = ShowMagneticField,
        };
    }
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

    protected override async Task OnAfterRenderAsync(bool firstRender)
    {
        if (firstRender)
        {
            await InitializeClientVisualization(SingleSimulationExtent);
            await RestartSimulation();
        }
    }

    private async Task RestartSimulation()
    {
        CancelSimulation();
        ResetSimulator();
        await UpdateClientVisualization();
        await InvokeAsync(StateHasChanged);
        await StartDrawSimulationLoop(_simLoopCancel.Token);
    }
    private void CancelSimulation()
    {
        _simLoopCancel.Cancel();
        _simLoopCancel.Dispose();
        _simLoopCancel = new CancellationTokenSource();
    }
    private void ResetSimulator()
    {
        FrameCount = 0;
        FrameRate = 0;

        //_simulationManager = new SimulationManager(GetSimulationParametersFromView());
        _collisionMagnetSimulator = new CollisionMagnetSimulation(GetSimulationParametersFromView());
        //_simulationManager.InitializeTwoMagnets();
        _collisionMagnetSimulator.InitializeTwoMagnets();
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
                    var parameters = GetSimulationParametersFromView();
                    //simulationManager.UpdateSimulation(parameters);
                    _collisionMagnetSimulator?.UpdateSimulation(parameters);
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

    private async Task InitializeClientVisualization(float extent)
    {
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs", 
            new { width = extent, height = extent, depth = extent },
            GetDrawingParametersFromView());

        _isVisualizationInitialized = true;
    }
    private async Task UpdateClientVisualization()
    {
        //if (_isVisualizationInitialized && _simulationManager != null)
        if (_isVisualizationInitialized && _collisionMagnetSimulator != null)
        {
            var state = _collisionMagnetSimulator.GetSimulationState();
            state.TimeSinceStart = FrameCount * TimeStep;

            var updateStopwatch = Stopwatch.StartNew();
            await JsRuntime.InvokeVoidAsync("Animations.updateThreeJsScene", state, GetDrawingParametersFromView());
            updateStopwatch.Stop();
            Profiling.LogMethodTime("JsRuntime.InvokeVoidAsync(Animations.updateThreeJsScene", updateStopwatch.ElapsedMilliseconds);
        }

        var stateChangedStopwatch = Stopwatch.StartNew();
        await InvokeAsync(StateHasChanged);
        stateChangedStopwatch.Stop();
        Profiling.LogMethodTime("InvokeAsync(StateHasChanged)", stateChangedStopwatch.ElapsedMilliseconds);
    }

    private async Task InstallApp()
    {
        var success = await JsRuntime.InvokeAsync<bool>("showPWAInstallPrompt");
        Console.WriteLine(
            $"Install prompt was {(success ? "" : "not")} accepted." +
            $"{(success ? "" : " Event to trigger it might not have been caught.")}");
    }
}