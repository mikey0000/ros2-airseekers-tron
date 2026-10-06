import {useTranslation} from "react-i18next";
import {useRobotProfile} from "./useRobotProfile.ts";

/**
 * The display name of the active robot, for the `{{robotName}}` i18n value
 * ("{{robotName}} is idle"). It is the active profile's translated label.
 * Strings that name the project (MowgliNext) or a vendor preset do not use it.
 *
 * Usage: `t("mowgliNextPage.mowgliMowing", {robotName})`.
 */
export function useRobotName(): string {
    const {t} = useTranslation();
    return t(useRobotProfile().profile.label);
}
