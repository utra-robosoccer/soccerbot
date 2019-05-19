import * as React from "react";
import "./index.scss";
import * as ROSLib from "roslib";
import { IInitialState } from "../../../reducers/initial-state";
import { cmdGlobalWrite } from "../../../utils/ros";
import classNames = require("classnames");

interface ICmdGlobalProps {
  cmdGlobal: IInitialState["cmdGlobal"];
}

class CmdGlobal extends React.Component<ICmdGlobalProps> {
  constructor(props: ICmdGlobalProps) {
    super(props);
  }

  private sendCmdGlobalMessage = (cmd: number) => {
    cmdGlobalWrite.publish(
      new ROSLib.Message({
        cmd
      })
    );
  };

  public render() {
    const { cmdGlobal } = this.props;

    return (
      <React.Fragment>
        <div className="cmd-global">
          <div className="cmd-global-pane">
            <div
              className={classNames({
                "cmd-global-pane-header": true,
                "cmd-global-pane-header-started": cmdGlobal === 0
              })}
            >
              {`CMD Global: ${cmdGlobal === 0 ? "Started" : "Stopped"}`}
            </div>
            <div className="cmd-global-pane-body">
              <div
                className="cmd-global-button cmd-global-button-start"
                onClick={() => this.sendCmdGlobalMessage(0)}
              >
                Start
              </div>
              <div
                className="cmd-global-button cmd-global-button-stop"
                onClick={() => this.sendCmdGlobalMessage(1)}
              >
                Stop
              </div>
            </div>
          </div>
        </div>
      </React.Fragment>
    );
  }
}

export default CmdGlobal;
