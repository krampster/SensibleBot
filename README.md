# RLBotCSharpExample

Example/template of a Rocket League bot implemented in C#

## Usage Instructions

### Prerequisites
Make sure you've installed [Python 3.7 64 bit](https://www.python.org/ftp/python/3.7.3/python-3.7.3-amd64.exe) or newer. During installation:
   - Select "Add Python to PATH"
   - Make sure pip is included in the installation

### Using Visual Studio
1. Install Visual Studio 2015 or newer. It should come with .NET Framework 4.6.1 or newer.
1. Open CSharpBot\Bot.sln in Visual Studio.
1. In Visual Studio, click the "Start" button, 
1. Double click on run-gui.bat.
1. Click the "Run" button. Rocket League should open automatically!

## Upgrades

This project uses a package manager called NuGet to keep track of the RLBot framework.
The framework will get updates periodically, and you'll probably want them, especially if you want to make sure
your bot will work right in the next tournament!

### Upgrading in Visual Studio
1. In Visual Studio, right click on the Bot C# project and choose "Manage NuGet Packages..."
1. Click on the "Installed" tab. You should see a package called "RLBot.Framework".
1. If an upgrade is available, it should say so and give you the option to upgrade.

## Notes

- Bot behavior is controlled by `CSharpBot/Bot/Bot.cs`
- Bot appearance is controlled by `PythonAgent/appearance.cfg`
- See the [wiki](https://github.com/RLBot/RLBotCSharpExample/wiki) for tips to improve your programming experience.
- If you'd like to keep up with bot strategies and bot tournaments, join our [Discord server](https://discord.gg/q9pbsWz). It's the heart of the RLBot community!


## Overview of how the C# bot interacts with Python

The C# bot executable is a server that listens for Python clients.
When `PythonAgent/PythonAgent.py` is started by the RLBot framework, it connects to the C# bot server and tells it its info.
Then, the C# bot server controls the bot through the `RLBot_Core_Interface` DLL.
