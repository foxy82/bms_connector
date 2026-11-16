"""Binary sensors decoded from Seplos V3 PIC status flags."""
from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .bms.seplos.v3.sensors import seplos_v3_device_info, status_flags_key
from .const import DOMAIN


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
        SeplosV3HeatingBinarySensor(
            context["coordinator"],
            entry_id=config_entry.entry_id,
            sensor_prefix=context["sensor_prefix"],
            modbus_address=addr,
            battery_address=context["battery_addresses"].get(addr),
            multi_pack=context["multi_pack"],
        )
        for addr in context["addresses"]
    )


class SeplosV3HeatingBinarySensor(CoordinatorEntity, BinarySensorEntity):
    """On while the pack heater runs (PIC FET event code, TB07 bit 3)."""

    _attr_device_class = BinarySensorDeviceClass.HEAT
    _attr_icon = "mdi:heating-coil"
    # Entity name already carries the prefix; prevent HA 2024+ doubling.
    _attr_has_entity_name = False

    def __init__(self, coordinator, entry_id, sensor_prefix, modbus_address,
                 battery_address, multi_pack):
        super().__init__(coordinator)
        self._modbus_address = modbus_address
        self._multi_pack = multi_pack
        address = battery_address or f"0x{modbus_address:02X}"
        self._attr_name = f"{sensor_prefix} - {address} - Heating"
        if multi_pack:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_{modbus_address}_Heating"
        else:
            self._attr_unique_id = f"bms_connector_v3_{entry_id}_Heating"
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
        return pic.heating if pic is not None else None
