//! The webcam appliance's menu (`uvc` builds), opened with the select button.
use num_traits::*;
use ux_api::menu::*;

use crate::ActionOp;
use crate::VaultOp;
use crate::webcam_ui::*;

pub fn create_submenu(vault_conn: xous::CID, actions_conn: xous::CID, menu_mgr: xous::SID) -> MenuMatic {
    let mut menu_items = Vec::<MenuItem>::new();
    let entries: [(&str, usize); 14] = [
        ("Exposure: auto", MENU_EXPOSURE_AUTO),
        ("Exposure: lock", MENU_EXPOSURE_LOCK),
        ("Exposure: manual", MENU_EXPOSURE_MANUAL),
        ("Gain +", MENU_GAIN_UP),
        ("Gain -", MENU_GAIN_DOWN),
        ("WB: auto", MENU_WB_AUTO),
        ("WB: calibrate", MENU_WB_CALIBRATE),
        ("View: preview", MENU_VIEW_FULL),
        ("View: zoom", MENU_VIEW_ZOOM),
        ("View: status", MENU_VIEW_STATUS),
        ("Rotate 180", MENU_ROTATE),
        ("Camera on/off", MENU_CAMERA),
        ("Save as default", MENU_SAVE),
        ("Reset USB", MENU_USB_RESET),
    ];
    for (name, action) in entries.iter() {
        menu_items.push(MenuItem {
            name: String::from(*name),
            action_conn: Some(vault_conn),
            action_opcode: VaultOp::WebcamMenu.to_u32().unwrap(),
            action_payload: MenuPayload::Scalar([*action as u32, 0, 0, 0]),
            close_on_select: true,
        });
    }
    menu_items.push(MenuItem {
        name: String::from("Close Menu"),
        action_conn: Some(actions_conn),
        action_opcode: ActionOp::MenuClose.to_u32().unwrap(),
        action_payload: MenuPayload::Scalar([0, 0, 0, 0]),
        close_on_select: true,
    });
    menu_items.push(MenuItem {
        name: String::from("Power Off"),
        action_conn: Some(vault_conn),
        action_opcode: VaultOp::PowerOff.to_u32().unwrap(),
        action_payload: MenuPayload::Scalar([0, 0, 0, 0]),
        close_on_select: true,
    });

    menu_matic(menu_items, "Webcam", Some(menu_mgr), vault_conn, VaultOp::MenuDone.to_usize().unwrap())
        .expect("couldn't create MenuMatic manager")
}
