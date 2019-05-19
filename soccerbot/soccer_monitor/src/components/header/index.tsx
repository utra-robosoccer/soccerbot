import * as React from "react";
import "./index.scss";
import CmdGlobal from "./cmd-global";
import { isNil } from "lodash";
import { IInitialState } from "../../reducers/initial-state";

interface IHeaderProps {
  cmdGlobal: IInitialState["cmdGlobal"];
  updateRobotSectionTopMargin: (arg: number) => void;
}

class Header extends React.PureComponent<IHeaderProps> {
  private readonly headerRef: React.RefObject<HTMLDivElement>;
  constructor(props: IHeaderProps) {
    super(props);
    this.headerRef = React.createRef();
  }
  public componentDidMount(): void {
    const { updateRobotSectionTopMargin } = this.props;
    if (isNil(this.headerRef.current)) {
      return;
    }
    updateRobotSectionTopMargin(this.headerRef.current.offsetHeight + 15);
  }

  public render() {
    return (
      <React.Fragment>
        <header className="header" ref={this.headerRef}>
          <div className="header-section">
            <img
              className="logo"
              src={require("../../assets/rapyuta_robotics_logo.png")}
            />
            <div className="header-name">Sootballs Monitor</div>
          </div>

          <div className="header-section">
            <CmdGlobal cmdGlobal={this.props.cmdGlobal} />
          </div>
        </header>
      </React.Fragment>
    );
  }
}

export default Header;
