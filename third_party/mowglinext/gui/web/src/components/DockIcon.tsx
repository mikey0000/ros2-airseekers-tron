import Icon from "@ant-design/icons";
import type {GetProps} from "antd";

// A charging dock (station with a lightning bolt), distinct from the house
// icon that the app uses for navigation "Home" and for "path to dock".
const DockSvg = () => (
    <svg viewBox="0 0 24 24" width="1em" height="1em" fill="none" stroke="currentColor"
         strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <path d="M4 20h16"/>
        <path d="M6 20V8a2 2 0 0 1 2-2h8a2 2 0 0 1 2 2v12"/>
        <path d="M12.5 9l-2 3.5h3l-2 3.5"/>
    </svg>
);

export const DockIcon = (props: Partial<GetProps<typeof Icon>>) => <Icon component={DockSvg} {...props}/>;
