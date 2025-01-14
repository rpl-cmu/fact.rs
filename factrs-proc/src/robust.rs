use quote::quote;
use syn::ItemImpl;

pub fn mark(item: ItemImpl) -> proc_macro2::TokenStream {
    if !cfg!(feature = "serde") {
        return quote! { #item };
    }

    quote! {
        #[typetag::serde]
        #item
    }
}
