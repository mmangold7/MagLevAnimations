using System.Diagnostics;

namespace Animations.Shared;

public static class Profiling
{
    // shell command for exposing api to network: "npx iisexpress-proxy https://localhost:7223 to 3000"

    public const bool ShouldLogMethodProfiles = true;

    public const string DebugEnvironmentName = "DEVELOPMENT";
    public const string ReleaseEnvironmentName = "PRODUCTION";

#if RELEASE
    public static string EnvironmentName => ReleaseEnvironmentName;
#else
    public static string EnvironmentName => DebugEnvironmentName;
#endif

    public static bool IsDebug = EnvironmentName == DebugEnvironmentName;
    public static bool IsRelease = EnvironmentName == ReleaseEnvironmentName;

    public static void RunWithClockingLog(Action action, string logText = null)
    {
        if (action == null) throw new ArgumentNullException(nameof(action));

        if (ShouldLogMethodProfiles && IsDebug)
        {
            logText = string.IsNullOrEmpty(logText) ? GetMethodDescription(action) : logText;

            var stopwatch = Stopwatch.StartNew();
            try
            {
                action();
            }
            catch (Exception ex)
            {
                // Log the exception if necessary
                Console.WriteLine($"Exception in {logText}: {ex.Message}");
                throw;
            }
            finally
            {
                stopwatch.Stop();
                LogMethodTime(logText, stopwatch.ElapsedMilliseconds);
            }
        }
        else
        {
            action();
        }
    }

    public static async Task RunWithClockingLogAsync(Func<Task> action, string logText = null)
    {
        if (action == null) throw new ArgumentNullException(nameof(action));

        if (ShouldLogMethodProfiles && IsDebug)
        {
            logText = string.IsNullOrEmpty(logText) ? GetMethodDescription(action) : logText;

            var stopwatch = Stopwatch.StartNew();
            try
            {
                await action();
            }
            catch (Exception ex)
            {
                // Log the exception if necessary
                Console.WriteLine($"Exception in {logText}: {ex.Message}");
                throw;
            }
            finally
            {
                stopwatch.Stop();
                LogMethodTime(logText, stopwatch.ElapsedMilliseconds);
            }
        }
        else
        {
            await action();
        }
    }

    private static string GetMethodDescription(Delegate action)
    {
        var methodInfo = action.Method;
        if (methodInfo.IsStatic || action.Target == null)
            return methodInfo.Name;

        var type = action.Target.GetType();
        return $"{type.FullName}.{methodInfo.Name}";
    }

    public static void LogMethodTime(string methodName, long elapsedMilliseconds) =>
        Console.WriteLine($"{methodName} took: {elapsedMilliseconds} ms");
}