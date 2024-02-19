using Microsoft.AspNetCore.Components;
using Microsoft.JSInterop;
using System.Numerics;
using Animations.Shared;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{
    [Inject]
    protected IJSRuntime? JsRuntime { get; set; }

    private SimulationManager? _simulationManager;

    private bool _showCoils;
    private bool _showGravityField;
    private bool _showMagneticField;
    private bool _useDipoleApproximation = true;

    private int _divisions = 10;
    private int _voxelsPerDivisions = 10;

    private DateTime _lastFrameTime = DateTime.UtcNow;
    private CancellationTokenSource _simLoopCancel = new();
    private int MaxFramesPerSecond { get; set; } = 30;
    private int FrameCount { get; set; }
    private float FrameRate { get; set; }
    private bool Paused { get; set; } = true;

    //protected override async Task OnAfterRenderAsync(bool firstRender)
    //{
    //    if (firstRender)
    //    {
    //        _simulationManager = new SimulationManager();
    //        _simulationManager.InitializeTwoMagnets();

    //        await InitializeVisualization(_simulationManager.SimulationExtents);
    //        await UpdateVisualization();
    //    }
    //}

    private async Task ToggleSimulation()
    {
        Paused = !Paused;
    }

    protected override async Task OnAfterRenderAsync(bool firstRender)
    {
        if (firstRender) await RestartSimulation();
    }

    private async Task RestartSimulation()
    {
        CancelSimulation();
        _simulationManager = await ResetSimAndGraphics(_simLoopCancel.Token);
        await UpdateUi(_simLoopCancel.Token);
        await StartSimulation(_simulationManager, _simLoopCancel.Token);
    }

    private void CancelSimulation()
    {
        _simLoopCancel.Cancel();
        _simLoopCancel.Dispose();
        _simLoopCancel = new CancellationTokenSource();
    }

    private async Task<SimulationManager> ResetSimAndGraphics(CancellationToken simLoopCancelToken)
    {
        FrameCount = 0;
        FrameRate = 0;

        var simulationManager = new SimulationManager();
        simulationManager.InitializeTwoMagnets();

        await InitializeVisualization(simulationManager.SimulationExtents);
        //await UpdateUi(simLoopCancelToken);
        return simulationManager;
    }

    private async Task StartSimulation(SimulationManager simulationManager, CancellationToken simLoopCancelToken)
    {
        while (!simLoopCancelToken.IsCancellationRequested)
        {
            if (Paused)
                await Task.Delay(100, simLoopCancelToken); // Small delay to reduce CPU usage

            else
                await Task.Run(async () =>
                {
                    simulationManager.UpdateSimulation();
                    await UpdateUi(simLoopCancelToken);
                }, simLoopCancelToken);
        }
    }

    private async Task UpdateUi(CancellationToken simLoopCancelToken)
    {
        await UpdateVisualization();
        await DelayUntilNextRequestedFrame(simLoopCancelToken);
        await InvokeAsync(StateHasChanged);
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

    private async Task ToggleVisualizationMode()
    {
        _showCoils = !_showCoils;
        await UpdateUi(_simLoopCancel.Token);
    }

    private async Task ToggleGravityField()
    {
        _showGravityField = !_showGravityField;
        _simulationManager.ShowGravityField = _showGravityField;
        _simulationManager.Divisions = _divisions;
        if (_showGravityField) _simulationManager.CalculateGravityField();
        await UpdateVisualization();
    }

    private async Task ToggleMagneticField()
    {
        _showMagneticField = !_showMagneticField;
        _simulationManager.ShowMagneticField = _showMagneticField;
        _simulationManager.Divisions = _divisions;
        if (_showMagneticField) _simulationManager.CalculateMagneticField();
        await UpdateVisualization();
    }

    private async Task ToggleApproximation()
    {
        _useDipoleApproximation = !_useDipoleApproximation;

        _simulationManager?.SwitchApproximationTo(_useDipoleApproximation
            ? SimulationMode.DipoleApproximation
            : SimulationMode.VoxelBased, _divisions, _voxelsPerDivisions);

        await UpdateVisualization();
    }

    private async Task InitializeVisualization(Vector3 extents) =>
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs", new
        {
            width = extents.X,
            height = extents.Y,
            depth = extents.Z
        });

    private async Task UpdateVisualization()
    {
        if (_simulationManager is { Magnets.Count: > 0 })
        {
            var magnets = _simulationManager.Magnets.Select(ConvertMagnetToJsObject).ToArray();
            var gravityField = _showGravityField
                ? _simulationManager.GravityFieldVectors.Select(ConvertFieldToJsObject).ToArray()
                : null;
            var magneticField = _showMagneticField
                ? _simulationManager.MagneticFieldVectors.Select(ConvertFieldToJsObject).ToArray()
                : null;
            await JsRuntime.InvokeVoidAsync("Animations.updateThreeJsScene", magnets, _showCoils, gravityField, magneticField);
        }
    }

    private object ConvertMagnetToJsObject(Magnet magnet) => new
    {
        position = new { magnet.Position.X, magnet.Position.Y, magnet.Position.Z },
        radius = magnet.Radius,
        length = magnet.Length,
        magnetization = new { magnet.Magnetization.X, magnet.Magnetization.Y, magnet.Magnetization.Z, },
    };

    private object ConvertFieldToJsObject(FieldVector vector) => new
    {
        position = new { vector.Position.X, vector.Position.Y, vector.Position.Z },
        direction = new { vector.Direction.X, vector.Direction.Y, vector.Direction.Z },
        magnitude = vector.Magnitude,
    };
}