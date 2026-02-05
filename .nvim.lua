-- Project-specific Neovim configuration

vim.lsp.config('yamlls',
  {
    settings = {
      yaml = {
        schemas = {
          ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_rtc/mc_rtc.json"] =
            {
              "**/mc_rtc.yaml",
              "etc/mc_rtc_r1_h2.yaml",
              "etc/mc_rtc_r2_h1.yaml"
            },
          ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMController.json"] = "etc/BiRobotTeleoperation.in.yaml",
          ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMStates.json"] = "src/states/data/*.yaml",
          ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMStates.json"] = {
            "etc/BiRobotTasks.in.yaml",
            "etc/BiRobotTasks.in.yaml",
            "etc/DampingTasks.in.yaml",
            "etc/ForcesDisplay.in.yaml",
            "etc/ForcesTasks.in.yaml",
            "etc/HalfSitting.in.yaml",
            "etc/HumanTasks.in.yaml",
            "etc/JointsDamping.in.yaml",
            "etc/RobotTeleopTasks.in.yaml",
            "etc/Stabilizer.in.yaml"
          },
          ["https://jrl.cnrs.fr/mc_rtc/schemas/Observers/ObserverPipelines.json"] = { "etc/Observers.in.yaml" },
          -- These files do not have json schemas yet
          -- "etc/HumanMap.in.yaml",
          -- "etc/mc_humanMap.in.yaml",
          -- "etc/log-to-datastore.in.yaml",
          -- "etc/RobotLimbMap.in.yaml",
          validate = true,
          format = { enable = false },
          hover = true,
          completion = true,
        }
      }
    }
  }
)
