-- Minimal subscriber seed for OAI CN5G (example values).
-- IMSI(ueid): 208950000000032
-- KEY (K):    fec86ba6eb707ed08905757b1bb44b8f
-- OPC:        C42449363BBAD02B66D16BC975D77CC1

INSERT INTO `AuthenticationSubscription`
(`ueid`, `authenticationMethod`, `encPermanentKey`, `protectionParameterId`,
 `sequenceNumber`, `authenticationManagementField`, `algorithmId`, `encOpcKey`,
 `encTopcKey`, `vectorGenerationInHss`, `n5gcAuthMethod`, `rgAuthenticationInd`, `supi`)
VALUES
('208950000000032', '5G_AKA',
 'fec86ba6eb707ed08905757b1bb44b8f',
 'fec86ba6eb707ed08905757b1bb44b8f',
 '{"sqn": "000000000000", "sqnScheme": "NON_TIME_BASED", "lastIndexes": {"ausf": 0}}',
 '8000',
 'milenage',
 'C42449363BBAD02B66D16BC975D77CC1',
 NULL, NULL, NULL, NULL,
 '001010000000001');
