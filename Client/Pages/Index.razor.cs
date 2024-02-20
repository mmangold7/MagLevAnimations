using Microsoft.AspNetCore.Components;
using System.Numerics;
using Animations.Shared;
using Microsoft.JSInterop;
using System.Diagnostics;
using System;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{
    private DateTime _lastFrameTime = DateTime.UtcNow;
    private CancellationTokenSource _simLoopCancel = new();
    private SimulationManager? _simulationManager;
    private bool _isVisualizationInitialized;

    private int Divisions { get; set; } = 10;
    private int VoxelsPerDivision { get; set; } = 10;
    private float TimeStep { get; set; } = 0.01f;
    private float Gravity { get; set; } = -9.81f;
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
    public string FieldDrawingType { get; set; } = "colorMapping";

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
        _simulationManager = await ResetSimulator(_simLoopCancel.Token);
        var currentState = _simulationManager.GetSimulationState();
        await SendCurrentStateToClientVisualization(currentState);
        await InvokeAsync(StateHasChanged);
        await StartDrawSimulationLoop(_simulationManager, _simLoopCancel.Token);
    }

    private void CancelSimulation()
    {
        _simLoopCancel.Cancel();
        _simLoopCancel.Dispose();
        _simLoopCancel = new CancellationTokenSource();
    }

    private async Task<SimulationManager> ResetSimulator(CancellationToken simLoopCancelToken)
    {
        FrameCount = 0;
        FrameRate = 0;

        var simulationManager = new SimulationManager(
            TimeStep,
            new Vector3(0, Gravity, 0),
            new Vector3(SingleSimulationExtent, SingleSimulationExtent, SingleSimulationExtent))
        {
            ShowGravityField = ShowGravityField,
            Divisions = Divisions,
            ShowMagneticField = ShowMagneticField
        };

        simulationManager.SwitchApproximationTo(UseDipoleApproximation
            ? SimulationMode.DipoleApproximation
            : SimulationMode.VoxelBased, Divisions, VoxelsPerDivision);

        simulationManager.InitializeTwoMagnets();

        //await UpdateUi(simLoopCancelToken);
        return simulationManager;
    }

    private async Task StartDrawSimulationLoop(SimulationManager simulationManager, CancellationToken simLoopCancelToken)
    {
        while (!simLoopCancelToken.IsCancellationRequested)
        {
            if (Paused)
                await Task.Delay(100, simLoopCancelToken); // Small delay to reduce CPU usage

            else
                await Task.Run(async () =>
                {
                    var newState = simulationManager.UpdateSimulation();
                    await SendCurrentStateToClientVisualization(newState);
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
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs", new
        {
            width = extent,
            height = extent,
            depth = extent
        });

        _isVisualizationInitialized = true;
    }

    private async Task SendCurrentStateToClientVisualization(SimulationState state)
    {
        if (_isVisualizationInitialized && _simulationManager is { Magnets.Count: > 0 })
        {
            //var stopwatch = Stopwatch.StartNew();
            //var (magnets, gravityField, magneticField) = MapMagnetsToJs();
            //stopwatch.Stop();
            //Profiling.LogMethodTime("MapMagnetsToJs", stopwatch.ElapsedMilliseconds);


            var stopwatch = Stopwatch.StartNew();
            await JsRuntime.InvokeVoidAsync("Animations.updateThreeJsScene",
                state.Magnets, ShowCoils, state.GravityFieldData, state.MagneticFieldData, FieldDrawingType);
            stopwatch.Stop();
            Profiling.LogMethodTime("JsRuntime.InvokeVoidAsync(Animations.updateThreeJsScene", stopwatch.ElapsedMilliseconds);

        }

        var stopwatch2 = Stopwatch.StartNew();
        await InvokeAsync(StateHasChanged);
        stopwatch2.Stop();
        Profiling.LogMethodTime("InvokeAsync(StateHasChanged)", stopwatch2.ElapsedMilliseconds);
    }

    private async Task InstallApp()
    {
        var success = await JsRuntime.InvokeAsync<bool>("showPWAInstallPrompt");
        Console.WriteLine(
            $"Install prompt was {(success ? "" : "not")} accepted." +
            $"{(success ? "" : " Event to trigger it might not have been caught.")}");
    }

    private async Task ToggleVisualizationMode()
    {
        ShowCoils = !ShowCoils;
        var currentState = _simulationManager.GetSimulationState();
        if (Paused) await SendCurrentStateToClientVisualization(currentState);
    }

    private async Task ToggleGravityField()
    {
        ShowGravityField = !ShowGravityField;
        _simulationManager.ShowGravityField = ShowGravityField;
        _simulationManager.Divisions = Divisions;
        if (ShowGravityField) _simulationManager.CalculateGravityField();
        var currentState = _simulationManager.GetSimulationState();
        if (Paused) await SendCurrentStateToClientVisualization(currentState);
    }

    private async Task ToggleMagneticField()
    {
        ShowMagneticField = !ShowMagneticField;
        _simulationManager.ShowMagneticField = ShowMagneticField;
        _simulationManager.Divisions = Divisions;
        if (ShowMagneticField) _simulationManager.CalculateMagneticField();
        var currentState = _simulationManager.GetSimulationState();
        if (Paused) await SendCurrentStateToClientVisualization(currentState);
    }

    private async Task ToggleApproximation()
    {
        UseDipoleApproximation = !UseDipoleApproximation;

        _simulationManager?.SwitchApproximationTo(UseDipoleApproximation
            ? SimulationMode.DipoleApproximation
            : SimulationMode.VoxelBased, Divisions, VoxelsPerDivision);

        var currentState = _simulationManager.GetSimulationState();
        await SendCurrentStateToClientVisualization(currentState);
    }

    private void TogglePaused() => Paused = !Paused;
    private void ToggleManualInputs() => ShowManualInputs = !ShowManualInputs;
    private void ToggleControlsVisibility() => AreControlsVisible = !AreControlsVisible;
    private string GetControlPanelClass() => AreControlsVisible ? "expanded" : "collapsed";
}