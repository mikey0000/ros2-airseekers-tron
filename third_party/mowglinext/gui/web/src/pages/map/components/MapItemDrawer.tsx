import React from "react";
import {Drawer} from "antd";
import {useIsMobile} from "../../../hooks/useIsMobile.ts";

/**
 * Sheet for one selected map item (no-go zone, boundary line): right-hand
 * panel on desktop, bottom drawer on mobile (same as AreaSettingsDrawer).
 * No mask, so the selection highlight stays visible and another map item can
 * be clicked straight away.
 */
export const MapItemDrawer: React.FC<{
    open: boolean;
    title: React.ReactNode;
    onClose: () => void;
    testId: string;
    children: React.ReactNode;
}> = ({open, title, onClose, testId, children}) => {
    const isMobile = useIsMobile();
    return (
        <Drawer
            open={open}
            onClose={onClose}
            placement={isMobile ? "bottom" : "right"}
            width={isMobile ? undefined : 360}
            height={isMobile ? "auto" : undefined}
            mask={false}
            title={title}
            destroyOnHidden
            styles={isMobile ? {wrapper: {maxHeight: "70vh"}} : undefined}
        >
            <div data-testid={testId}>{children}</div>
        </Drawer>
    );
};
