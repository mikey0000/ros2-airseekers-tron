import {Button, Result} from "antd";
import {useTranslation} from "react-i18next";
import {isRouteErrorResponse, useLocation, useNavigate, useRouteError} from "react-router-dom";

/**
 * Router fallbacks. Without these React Router renders its developer-facing
 * "Unexpected Application Error!" screen for unknown paths (e.g. #/stats) and
 * for anything a page throws while loading — including the "Failed to fetch
 * dynamically imported module" / "Unable to preload CSS" errors a browser hits
 * when it still holds the previous build's index after the GUI is updated.
 */

/** Rendered by the catch-all `*` route, inside the app shell. */
export function NotFoundPage() {
    const {t} = useTranslation();
    const navigate = useNavigate();
    const {pathname} = useLocation();
    return (
        <Result
            status="404"
            title={t("routeError.notFoundTitle")}
            subTitle={t("routeError.notFoundBody", {path: pathname})}
            extra={<Button type="primary" onClick={() => void navigate("/mowglinext")}>{t("routeError.goHome")}</Button>}
        />
    );
}

/** `errorElement` for the shell and every page route. */
export function RouteErrorPage() {
    const {t} = useTranslation();
    const error = useRouteError();
    if (isRouteErrorResponse(error) && error.status === 404) {
        return <NotFoundPage/>;
    }
    const detail = isRouteErrorResponse(error)
        ? `${error.status} ${error.statusText}`
        : error instanceof Error ? error.message : undefined;
    return (
        <Result
            status="error"
            title={t("routeError.errorTitle")}
            subTitle={<>{t("routeError.errorBody")}{detail && <><br/><code>{detail}</code></>}</>}
            extra={[
                <Button key="reload" type="primary" onClick={() => window.location.reload()}>{t("routeError.reload")}</Button>,
                // A plain hash link: this element may render outside the shell
                // when the shell itself failed, so avoid depending on its state.
                <Button key="home" href="#/mowglinext">{t("routeError.goHome")}</Button>,
            ]}
        />
    );
}
