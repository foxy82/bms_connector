"""Binary sensors decoded from Seplos V3 PIC status flags."""
from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .bms.seplos.v3.sensors import seplos_v3_device_info, status_flags_key
from .const import DOMAIN

# (name, V3PICTableData attribute, device class, icon, diagnostic,
#  attribute listing the active items as an extra state attribute)
BINARY_SENSORS = (
    ("Heating", "heating", None, "mdi:heating-coil", False, None),
    ("Charging", "charging", BinarySensorDeviceClass.BATTERY_CHARGING, None, False, None),
    ("Discharging", "discharging", None, "mdi:battery-arrow-down", False, None),
    ("Alarm", "active_alarms", BinarySensorDeviceClass.PROBLEM, None, False, "alarms"),
    ("Fault", "active_faults", BinarySensorDeviceClass.PROBLEM, None, False, "faults"),
    ("Charge FET", "charge_fet", None, "mdi:electric-switch",
     True, None),
    ("Discharge FET", "discharge_fet", None, "mdi:electric-switch",
     True, None),
    ("Current Limiting", "current_limiting", None, "mdi:speedometer-slow", True, None),
)


async def async_setup_entry(hass: HomeAssistant, config_entry: ConfigEntry,
                            async_add_entities: AddEntitiesCallback):
    """Add PIC-derived binary sensors when the entry reads status flags.

    The sensor platform creates the coordinator and is set up first; it only
    publishes a status-flags context for Seplos V3 entries with
    read_status_flags enabled, so other entries get no entities here.
    """
    context = hass.data.get(DOMAIN, {}).get(status_flags_key(config_entry.entry_id))
    if context is None:
        return

    async_add_entities(
        SeplosV3StatusBinarySensor(
            context["coordinator"],
            description,
            entry_id=config_entry.entry_id,
            sensor_prefix=context["sensor_prefix"],
            modbus_address=addr,
            battery_address=context["battery_addresses"].get(addr),
            multi_pack=context["multi_pack"],
        )
        for addr in context["addresses"]
        for description in BINARY_SENSORS
    )


class SeplosV3StatusBinarySensor(CoordinatorEntity, BinarySensorEntity):
    """One flag from a pack's PIC status table.

    A list-valued attribute (alarms, faults) is on while the list is
    non-empty and exposes the list as an extra state attribute.
    """

    # Entity name already carries the prefix; prevent HA 2024+ doubling.
    _attr_has_entity_name = False

    def __init__(self, coordinator, description, entry_id, sensor_prefix,
                 modbus_address, battery_address, multi_pack):
        super().__init__(coordinator)
        name, self._flag, device_class, icon, diagnostic, self._list_attr = description
        self._modbus_address = modbus_address
        self._multi_pack = multi_pack
        self._attr_device_class = device_class
        if icon:
            self._attr_icon = icon
        if diagnostic:
            self._attr_entity_category = EntityCategory.DIAGNOSTIC
        address = battery_address or f"0x{modbus_address:02X}"
        self._attr_name = f"{sensor_prefix} - {address} - {name}"
        if multi_pack:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_{modbus_address}_{name}"
        else:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_{name}"
        self._attr_device_info = seplos_v3_device_info(
            entry_id, sensor_prefix, modbus_address, multi_pack
        )

    def _pic(self):
        data = self.coordinator.data
        if self._multi_pack and isinstance(data, dict):
            data = data.get(self._modbus_address)
        if isinstance(data, tuple) and len(data) > 5:
            return data[5]
        return None

    @property
    def available(self) -> bool:
        return super().available and self._pic() is not None

    @property
    def is_on(self):
        pic = self._pic()
        return bool(getattr(pic, self._flag)) if pic is not None else None

    @property
    def extra_state_attributes(self):
        pic = self._pic()
        if self._list_attr is None or pic is None:
            return None
        return {self._list_attr: getattr(pic, self._flag)}
